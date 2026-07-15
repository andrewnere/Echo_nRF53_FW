/*
 * imu_mouse.c — IMU-to-mouse-delta processor for LSM6DSO on nRF5340
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "imu.h"
#include "imu_mouse.h"
#include "app_config.h"

LOG_MODULE_REGISTER(imu_mouse, LOG_LEVEL_INF);

/* Radians ↔ degrees */
#define RAD2DEG (180.0f / 3.14159265358979323846f)
#define DEG2RAD (3.14159265358979323846f / 180.0f)

/* ============================================================
 * Internal state — single static instance, no heap
 * ============================================================ */

/* ---------- Calibration accumulators ---------- */
static int32_t  s_cal_skip;            /* samples discarded during settle   */
static int32_t  s_cal_count;           /* samples collected so far          */
static double   s_accel_sum[3];        /* running sum for gravity estimate  */
static double   s_gyro_sum[3];         /* running sum for bias estimate     */
static bool     s_calibrated;          /* true once IMU_CAL_SAMPLES reached     */

/* ---------- Calibration results ---------- */
static float    s_gyro_bias[3];        /* raw-count bias per axis           */
static float    s_gravity[3];          /* normalised gravity vector (world) */

/*
 * 3×3 remap matrix (row-major).
 * Maps corrected sensor-frame vector → world frame.
 * Initialised to identity; overwritten after calibration.
 */
static float    s_remap[3][3];

/* ---------- Accel EMA state ---------- */
static float    s_accel_lpf[3];        /* filtered accel in raw counts      */
static bool     s_accel_lpf_seeded;

/* ---------- GRV quaternion [W, X, Y, Z] ----------
 * Named s_qEstGRV / qEstGRV to match the Game Rotation Vector (GRV) section
 * of the author's own fusion.c (SensorFusion/), for easy cross-reference. */
static float    s_qEstGRV[4];

/* ---------- Mouse delta state ---------- */
static float    s_prev_yaw;
static float    s_prev_pitch;
static bool     s_mouse_seeded;        /* true once prev_yaw/pitch are set  */
static float    s_smooth_yaw;
static float    s_smooth_pitch;

/* Sub-pixel accumulators */
static float    s_accum_x;
static float    s_accum_y;

/* ---------- Output callback ---------- */
static mouse_output_cb_t s_output_cb;

/* ---------- Runtime enable flag ---------- */
static bool s_mouse_enabled;

/* ---------- dt smoothing ---------- */
static uint32_t s_prev_ts;
static bool     s_ts_seeded;
static float    s_smoothed_dt;

/* ============================================================
 * dt computation from hardware ticks
 *
 * Computes the inter-sample period in seconds.  Uses uint32 subtraction
 * for correct wrap-around handling on the 32-bit tick counter.
 * ============================================================ */
static float compute_dt(uint32_t ts_ticks) {
    if (!s_ts_seeded) {
        s_prev_ts = ts_ticks;
        s_ts_seeded = true;
        s_smoothed_dt = IMU_NOMINAL_DT;
        return IMU_NOMINAL_DT;
    }

    uint32_t delta = ts_ticks - s_prev_ts; /* wraps correctly for uint32 */
    s_prev_ts = ts_ticks;

    float measured = (float)delta * IMU_TS_UNIT_S;

    if (measured < IMU_MIN_DT || measured > IMU_MAX_DT) {
        /* Outlier — clamp at nominal */
        return IMU_NOMINAL_DT;
    }

    s_smoothed_dt = IMU_DT_ALPHA_NEW * measured + IMU_DT_ALPHA_OLD * s_smoothed_dt;
    return s_smoothed_dt;
}

/* Apply the remap matrix to a 3-vector (in-place). */
static void apply_remap(float v[3]) {
    float out[3];
    out[0] = s_remap[0][0]*v[0] + s_remap[0][1]*v[1] + s_remap[0][2]*v[2];
    out[1] = s_remap[1][0]*v[0] + s_remap[1][1]*v[1] + s_remap[1][2]*v[2];
    out[2] = s_remap[2][0]*v[0] + s_remap[2][1]*v[1] + s_remap[2][2]*v[2];
    v[0] = out[0];
    v[1] = out[1];
    v[2] = out[2];
}

/* ============================================================
 * initQuaternionGRV — reset the GRV quaternion to identity.
 * ============================================================ */
static void initQuaternionGRV(void) {
    s_qEstGRV[0] = 1.0f; s_qEstGRV[1] = 0.0f; s_qEstGRV[2] = 0.0f; s_qEstGRV[3] = 0.0f;
}

/* ============================================================
 * updateQuaternionGRV — Game Rotation Vector update (gradient-descent
 * accel+gyro fusion, no magnetometer). 
 *
 * Inputs:
 *   gx_dps, gy_dps, gz_dps — gyro in degrees per second
 *   ax_g,   ay_g,   az_g   — accel in g (already divided by IMU_ACCEL_SCALE)
 *   dt                     — integration step in seconds (gameDT)
 *
 * Updates s_qEstGRV[] in-place.  Quaternion convention: [W, X, Y, Z].
 * ============================================================ */
static void updateQuaternionGRV(float gx_dps, float gy_dps, float gz_dps,
                                float ax_g,   float ay_g,   float az_g,
                                float dt) {
    float qEstGRV_W = s_qEstGRV[0], qEstGRV_X = s_qEstGRV[1],
          qEstGRV_Y = s_qEstGRV[2], qEstGRV_Z = s_qEstGRV[3];

    /* --- Normalise accelerometer (qAcc in the original) --- */
    float qAcc_norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    if (qAcc_norm < 1e-10f) {
        return; /* free-fall or bad data — skip this update */
    }
    float qAcc_X = ax_g / qAcc_norm;
    float qAcc_Y = ay_g / qAcc_norm;
    float qAcc_Z = az_g / qAcc_norm;

    /* --- Gradient descent objective function (objFunctionG) --- */
    float objFunctionG_X = 2.0f*(qEstGRV_X*qEstGRV_Z - qEstGRV_W*qEstGRV_Y) - qAcc_X;
    float objFunctionG_Y = 2.0f*(qEstGRV_W*qEstGRV_X + qEstGRV_Y*qEstGRV_Z) - qAcc_Y;
    float objFunctionG_Z = 1.0f - 2.0f*(qEstGRV_X*qEstGRV_X + qEstGRV_Y*qEstGRV_Y) - qAcc_Z;

    /* --- Jacobian transpose times objFunctionG (qDelF = Jg^T · objFunctionG) --- */
    float qDelF_W = -2.0f*qEstGRV_Y*objFunctionG_X + 2.0f*qEstGRV_X*objFunctionG_Y;
    float qDelF_X =  2.0f*qEstGRV_Z*objFunctionG_X + 2.0f*qEstGRV_W*objFunctionG_Y - 4.0f*qEstGRV_X*objFunctionG_Z;
    float qDelF_Y = -2.0f*qEstGRV_W*objFunctionG_X + 2.0f*qEstGRV_Z*objFunctionG_Y - 4.0f*qEstGRV_Y*objFunctionG_Z;
    float qDelF_Z =  2.0f*qEstGRV_X*objFunctionG_X + 2.0f*qEstGRV_Y*objFunctionG_Y;

    /* --- Normalise qDelF --- */
    float qDelF_norm = sqrtf(qDelF_W*qDelF_W + qDelF_X*qDelF_X + qDelF_Y*qDelF_Y + qDelF_Z*qDelF_Z);
    if (qDelF_norm > 1e-10f) {
        qDelF_W /= qDelF_norm;
        qDelF_X /= qDelF_norm;
        qDelF_Y /= qDelF_norm;
        qDelF_Z /= qDelF_norm;
    }

    /* --- Gyro to radians per second (qGyro in the original) --- */
    float qGyro_X = gx_dps * DEG2RAD;
    float qGyro_Y = gy_dps * DEG2RAD;
    float qGyro_Z = gz_dps * DEG2RAD;


    float betaGRV = IMU_MADGWICK_BETA;
    float qGyroDerivative_W = 0.5f*(-qEstGRV_X*qGyro_X - qEstGRV_Y*qGyro_Y - qEstGRV_Z*qGyro_Z) - betaGRV*qDelF_W;
    float qGyroDerivative_X = 0.5f*( qEstGRV_W*qGyro_X + qEstGRV_Y*qGyro_Z - qEstGRV_Z*qGyro_Y) - betaGRV*qDelF_X;
    float qGyroDerivative_Y = 0.5f*( qEstGRV_W*qGyro_Y - qEstGRV_X*qGyro_Z + qEstGRV_Z*qGyro_X) - betaGRV*qDelF_Y;
    float qGyroDerivative_Z = 0.5f*( qEstGRV_W*qGyro_Z + qEstGRV_X*qGyro_Y - qEstGRV_Y*qGyro_X) - betaGRV*qDelF_Z;

    /* --- Integrate --- */
    qEstGRV_W += qGyroDerivative_W * dt;
    qEstGRV_X += qGyroDerivative_X * dt;
    qEstGRV_Y += qGyroDerivative_Y * dt;
    qEstGRV_Z += qGyroDerivative_Z * dt;

    /* --- Normalise quaternion --- */
    float qEstGRV_norm = sqrtf(qEstGRV_W*qEstGRV_W + qEstGRV_X*qEstGRV_X
                              + qEstGRV_Y*qEstGRV_Y + qEstGRV_Z*qEstGRV_Z);
    if (qEstGRV_norm < 1e-10f) {
        initQuaternionGRV(); /* degenerate — reset to identity */
        return;
    }
    s_qEstGRV[0] = qEstGRV_W / qEstGRV_norm;
    s_qEstGRV[1] = qEstGRV_X / qEstGRV_norm;
    s_qEstGRV[2] = qEstGRV_Y / qEstGRV_norm;
    s_qEstGRV[3] = qEstGRV_Z / qEstGRV_norm;
}

/* ============================================================
 * Euler angles from quaternion (degrees)
 * ============================================================ */
static void convertQuaternionToEuler(float *roll, float *pitch, float *yaw) {
    float qEstGRV_W = s_qEstGRV[0], qEstGRV_X = s_qEstGRV[1],
          qEstGRV_Y = s_qEstGRV[2], qEstGRV_Z = s_qEstGRV[3];

    *roll  = atan2f(2.0f*(qEstGRV_W*qEstGRV_X + qEstGRV_Y*qEstGRV_Z),
                    1.0f - 2.0f*(qEstGRV_X*qEstGRV_X + qEstGRV_Y*qEstGRV_Y)) * RAD2DEG;

    float sinp = 2.0f*(qEstGRV_W*qEstGRV_Y - qEstGRV_Z*qEstGRV_X);
    /* clamp to [-1, 1] before asin to avoid NaN at ±90° */
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * RAD2DEG;

    *yaw   = atan2f(2.0f*(qEstGRV_W*qEstGRV_Z + qEstGRV_X*qEstGRV_Y),
                    1.0f - 2.0f*(qEstGRV_Y*qEstGRV_Y + qEstGRV_Z*qEstGRV_Z)) * RAD2DEG;
}

/* ============================================================
 * Calibration phase
 *
 * Called for each of the first IMU_CAL_SAMPLES samples.  Accumulates gyro and
 * accel sums, runs updateQuaternionGRV() 
 * Raw accel/gyro inputs are in raw LSB counts (not yet converted to g / dps).
 * ============================================================ */
static void calibration_update(float ax_raw, float ay_raw, float az_raw,
                               float gx_raw, float gy_raw, float gz_raw,
                               float dt) {
    /* --- Accel EMA LPF (runs throughout so it's warm when collection starts) --- */
    if (!s_accel_lpf_seeded) {
        s_accel_lpf[0] = ax_raw;
        s_accel_lpf[1] = ay_raw;
        s_accel_lpf[2] = az_raw;
        s_accel_lpf_seeded = true;
    } else {
        s_accel_lpf[0] = IMU_ACCEL_LPF_ALPHA*ax_raw + (1.0f - IMU_ACCEL_LPF_ALPHA)*s_accel_lpf[0];
        s_accel_lpf[1] = IMU_ACCEL_LPF_ALPHA*ay_raw + (1.0f - IMU_ACCEL_LPF_ALPHA)*s_accel_lpf[1];
        s_accel_lpf[2] = IMU_ACCEL_LPF_ALPHA*az_raw + (1.0f - IMU_ACCEL_LPF_ALPHA)*s_accel_lpf[2];
    }

    /* --- Skip phase: let sensor output settle before accumulating bias --- */
    if (s_cal_skip < IMU_CAL_SKIP_SAMPLES) {
        s_cal_skip++;
#ifdef ENABLE_UART_DEBUGGING
        if (s_cal_skip == 1 || s_cal_skip == IMU_CAL_SKIP_SAMPLES) {
            printk("[cal] settling %d/%d\n", s_cal_skip, IMU_CAL_SKIP_SAMPLES);
        }
#endif
        /* Run updateQuaternionGRV during skip so it converges before collection ends */
        float gx_dps = gx_raw / IMU_GYRO_SCALE;
        float gy_dps = gy_raw / IMU_GYRO_SCALE;
        float gz_dps = gz_raw / IMU_GYRO_SCALE;
        updateQuaternionGRV(gx_dps, gy_dps, gz_dps,
                            s_accel_lpf[0] / IMU_ACCEL_SCALE,
                            s_accel_lpf[1] / IMU_ACCEL_SCALE,
                            s_accel_lpf[2] / IMU_ACCEL_SCALE, dt);
        return;
    }

    /* --- Accumulate for bias / gravity estimates --- */
    s_gyro_sum[0]  += (double)gx_raw;
    s_gyro_sum[1]  += (double)gy_raw;
    s_gyro_sum[2]  += (double)gz_raw;
    s_accel_sum[0] += (double)ax_raw;
    s_accel_sum[1] += (double)ay_raw;
    s_accel_sum[2] += (double)az_raw;

    /* --- Run updateQuaternionGRV with filtered accel so it converges --- */
    float gx_dps = gx_raw / IMU_GYRO_SCALE;
    float gy_dps = gy_raw / IMU_GYRO_SCALE;
    float gz_dps = gz_raw / IMU_GYRO_SCALE;
    float ax_g   = s_accel_lpf[0] / IMU_ACCEL_SCALE;
    float ay_g   = s_accel_lpf[1] / IMU_ACCEL_SCALE;
    float az_g   = s_accel_lpf[2] / IMU_ACCEL_SCALE;
    updateQuaternionGRV(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt);

    s_cal_count++;

    if (s_cal_count < IMU_CAL_SAMPLES) {
        return; /* still collecting */
    }

    /* --- Calibration complete: compute bias + gravity --- */
    s_gyro_bias[0] = (float)(s_gyro_sum[0]  / IMU_CAL_SAMPLES);
    s_gyro_bias[1] = (float)(s_gyro_sum[1]  / IMU_CAL_SAMPLES);
    s_gyro_bias[2] = (float)(s_gyro_sum[2]  / IMU_CAL_SAMPLES);

    float g[3];
    g[0] = (float)(s_accel_sum[0] / IMU_CAL_SAMPLES);
    g[1] = (float)(s_accel_sum[1] / IMU_CAL_SAMPLES);
    g[2] = (float)(s_accel_sum[2] / IMU_CAL_SAMPLES);

    /* Sanity: need a believable gravity magnitude */
    float gmag = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
    if (gmag < IMU_MIN_GRAVITY_LSB) {
        LOG_ERR("Calibration failed: gravity magnitude %.1f LSB < %.0f — retrying",
                (double)gmag, (double)IMU_MIN_GRAVITY_LSB);
        /* Reset accumulators, keep running */
        s_cal_count = 0;
        memset(s_gyro_sum,  0, sizeof(s_gyro_sum));
        memset(s_accel_sum, 0, sizeof(s_accel_sum));
        return;
    }

    /* Normalise gravity and store */
    s_gravity[0] = g[0] / gmag;
    s_gravity[1] = g[1] / gmag;
    s_gravity[2] = g[2] / gmag;

    /* Set sensor→world remap matrix */
#if defined(IMU_MOUNT_Y_DOWN)
    /* Board rotated 90° around X-axis; Y-axis pointing down.
     * world_X = sensor_X, world_Y = sensor_Z, world_Z = -sensor_Y */
    memset(s_remap, 0, sizeof(s_remap));
    s_remap[0][0] =  1.0f;
    s_remap[1][2] =  1.0f;
    s_remap[2][1] = -1.0f;
    LOG_INF("Mounting: Y-down (hardcoded)");
#elif defined(IMU_MOUNT_Z_UP)
    /* Board flat, Z-axis pointing up — identity remap. */
    memset(s_remap, 0, sizeof(s_remap));
    s_remap[0][0] = 1.0f;
    s_remap[1][1] = 1.0f;
    s_remap[2][2] = 1.0f;
    LOG_INF("Mounting: Z-up (hardcoded)");
#else
#error "Define IMU_MOUNT_Y_DOWN or IMU_MOUNT_Z_UP in app_config.h"
#endif

    s_calibrated = true;

    LOG_INF("Calibration done: bias=[%.2f, %.2f, %.2f] LSB, gravity=[%.3f, %.3f, %.3f]",
            (double)s_gyro_bias[0], (double)s_gyro_bias[1], (double)s_gyro_bias[2],
            (double)s_gravity[0],   (double)s_gravity[1],   (double)s_gravity[2]);
}

/* ============================================================
 * Post-calibration sensor fusion
 *
 * Applies gyro bias correction and axis remap, runs the accel LPF and
 * updateQuaternionGRV, then converts the quaternion to Euler angles.
 * ============================================================ */
static void fusion_update(float ax_raw, float ay_raw, float az_raw,
                          float gx_raw, float gy_raw, float gz_raw,
                          float dt,
                          float *yaw_out, float *pitch_out) {
    /* --- Gyro bias correction --- */
    float gx_corr = gx_raw - s_gyro_bias[0];
    float gy_corr = gy_raw - s_gyro_bias[1];
    float gz_corr = gz_raw - s_gyro_bias[2];

#ifdef ENABLE_UART_DEBUGGING
    /* Print mean corrected gyro every second — reveals true residual bias.
     * Single-sample noise is ±15 LSB so the mean is far more informative. */
    static int32_t s_gyro_dbg;
    static int32_t s_gx_acc, s_gy_acc, s_gz_acc;
    s_gx_acc += (int32_t)gx_corr;
    s_gy_acc += (int32_t)gy_corr;
    s_gz_acc += (int32_t)gz_corr;
    if (++s_gyro_dbg >= IMU_ODR_HZ) {
        printk("[gyro] bias=%d,%d,%d  mean_corr=%d,%d,%d (x10, 1s avg)\n",
               (int)s_gyro_bias[0], (int)s_gyro_bias[1], (int)s_gyro_bias[2],
               s_gx_acc * 10 / IMU_ODR_HZ, s_gy_acc * 10 / IMU_ODR_HZ, s_gz_acc * 10 / IMU_ODR_HZ);
        s_gyro_dbg = 0;
        s_gx_acc = s_gy_acc = s_gz_acc = 0;
    }
#endif

    /* --- Accel EMA LPF --- */
    s_accel_lpf[0] = IMU_ACCEL_LPF_ALPHA*ax_raw + (1.0f - IMU_ACCEL_LPF_ALPHA)*s_accel_lpf[0];
    s_accel_lpf[1] = IMU_ACCEL_LPF_ALPHA*ay_raw + (1.0f - IMU_ACCEL_LPF_ALPHA)*s_accel_lpf[1];
    s_accel_lpf[2] = IMU_ACCEL_LPF_ALPHA*az_raw + (1.0f - IMU_ACCEL_LPF_ALPHA)*s_accel_lpf[2];

    /* --- Remap both vectors from sensor frame to world frame --- */
    float g_vec[3] = { s_accel_lpf[0], s_accel_lpf[1], s_accel_lpf[2] };
    float r_vec[3] = { gx_corr,        gy_corr,        gz_corr        };
    apply_remap(g_vec);
    apply_remap(r_vec);

    /* --- updateQuaternionGRV (gyro in dps, accel in g) --- */
    float gx_dps = r_vec[0] / IMU_GYRO_SCALE;
    float gy_dps = r_vec[1] / IMU_GYRO_SCALE;
    float gz_dps = r_vec[2] / IMU_GYRO_SCALE;
    float ax_g   = g_vec[0] / IMU_ACCEL_SCALE;
    float ay_g   = g_vec[1] / IMU_ACCEL_SCALE;
    float az_g   = g_vec[2] / IMU_ACCEL_SCALE;
    updateQuaternionGRV(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt);

    /* Extract Euler angles: yaw → cursor X, pitch → cursor Y, roll discarded. */
    float roll_unused;
    convertQuaternionToEuler(&roll_unused, pitch_out, yaw_out);
    (void)roll_unused;
}

/* ============================================================
 * Mouse delta computation (Stage 2)
 *
 * Rate-independent EMA smoothing.
 * Writes integer dx/dy into *out_dx and *out_dy via sub-pixel accumulation.
 * Returns true if at least one axis is non-zero.
 * ============================================================ */
static bool mouse_delta(float yaw, float pitch, float dt,
                        int8_t *out_dx, int8_t *out_dy) {
    /* First call: seed the previous state with the current angles */
    if (!s_mouse_seeded) {
        s_prev_yaw   = yaw;
        s_prev_pitch = pitch;
        s_smooth_yaw   = yaw;
        s_smooth_pitch = pitch;
        s_mouse_seeded = true;
        *out_dx = 0;
        *out_dy = 0;
        return false;
    }

    /* --- Rate-independent EMA --- */
    float alpha = 1.0f - expf(-dt / MOUSE_SMOOTH_TAU);
    s_smooth_yaw   = alpha*yaw   + (1.0f - alpha)*s_prev_yaw;
    s_smooth_pitch = alpha*pitch + (1.0f - alpha)*s_prev_pitch;

    /* --- Angular deltas from sticky reference --- */
    float dx = s_smooth_yaw   - s_prev_yaw;
    float dy = s_smooth_pitch - s_prev_pitch;

    /* Wrap deltas to [-180, +180] to survive the ±180° yaw discontinuity.
     * Without this, crossing the boundary causes a ~360° spike that fills
     * the sub-pixel accumulator and locks up the cursor. */
    if (dx >  180.0f) dx -= 360.0f;
    if (dx < -180.0f) dx += 360.0f;
    if (dy >  180.0f) dy -= 360.0f;
    if (dy < -180.0f) dy += 360.0f;

    /* --- Deadzone gate and sensitivity --- */
    bool moved = false;

    if (fabsf(dx) < MOUSE_DEADZONE_X) {
        dx = 0.0f;
    } else {
        dx *= -MOUSE_SENSITIVITY_X;          /* negated: sensor yaw is inverted vs screen X */
        s_prev_yaw = s_smooth_yaw;
        moved = true;
    }

    if (fabsf(dy) < MOUSE_DEADZONE_Y) {
        dy = 0.0f;
    } else {
        dy *= MOUSE_SENSITIVITY_Y;
        s_prev_pitch = s_smooth_pitch;
        moved = true;
    }

    /* --- Sub-pixel accumulation → int8 HID output --- */
    s_accum_x += dx;
    s_accum_y += dy;

    /* Clamp to int8 range */
    float rep_xf = s_accum_x;
    float rep_yf = s_accum_y;
    if (rep_xf >  127.0f) rep_xf =  127.0f;
    if (rep_xf < -127.0f) rep_xf = -127.0f;
    if (rep_yf >  127.0f) rep_yf =  127.0f;
    if (rep_yf < -127.0f) rep_yf = -127.0f;

    int8_t rep_x = (int8_t)rep_xf;
    int8_t rep_y = (int8_t)rep_yf;

    /* Carry the unconsumed fractional remainder forward */
    s_accum_x -= (float)rep_x;
    s_accum_y -= (float)rep_y;

    *out_dx = rep_x;
    *out_dy = rep_y;

    return moved && (rep_x != 0 || rep_y != 0);
}

/* ============================================================
 * Public API
 * ============================================================ */

static bool imu_mouse_update(const struct imu_sample *s, int8_t *dx, int8_t *dy);

/* Internal IMU sample handler — registered once via imu_add_sample_cb. */
static void on_imu_sample(const struct imu_sample *s) {
    if (!s_mouse_enabled) {
        return;
    }

    int8_t dx, dy;
    bool moved = imu_mouse_update(s, &dx, &dy);

#ifdef ENABLE_UART_DEBUGGING
    /* Print once per second. ×10 scaling gives 1 decimal place without %f. */
    static int32_t s_dbg_count;
    if (++s_dbg_count >= IMU_ODR_HZ) {
        s_dbg_count = 0;
        if (!s_calibrated) {
            if (s_cal_skip < IMU_CAL_SKIP_SAMPLES) {
                printk("[cal] settling %d/%d\n", s_cal_skip, IMU_CAL_SKIP_SAMPLES);
            } else {
                printk("[cal] collecting %d/%d\n", s_cal_count, IMU_CAL_SAMPLES);
            }
        } else {
            float roll, pitch, yaw;
            convertQuaternionToEuler(&roll, &pitch, &yaw);
            printk("[imu] q=[%d,%d,%d,%d]/1k yaw=%d pitch=%d roll=%d dx=%d dy=%d\n",
                   (int)(s_qEstGRV[0] * 1000), (int)(s_qEstGRV[1] * 1000),
                   (int)(s_qEstGRV[2] * 1000), (int)(s_qEstGRV[3] * 1000),
                   (int)(yaw * 10), (int)(pitch * 10), (int)(roll * 10),
                   (int)dx, (int)dy);
        }
    }
#endif

    if (moved && s_output_cb) {
        s_output_cb(dx, dy);
    }
}

/* Reset all algorithm state without touching s_output_cb or re-registering. */
static void reset_state(void) {
    s_cal_skip         = 0;
    s_cal_count        = 0;
    s_calibrated       = false;
    s_ts_seeded        = false;
    s_mouse_seeded     = false;
    s_accel_lpf_seeded = false;

    memset(s_accel_sum,  0, sizeof(s_accel_sum));
    memset(s_gyro_sum,   0, sizeof(s_gyro_sum));
    memset(s_gyro_bias,  0, sizeof(s_gyro_bias));
    memset(s_gravity,    0, sizeof(s_gravity));
    memset(s_accel_lpf,  0, sizeof(s_accel_lpf));

    memset(s_remap, 0, sizeof(s_remap));
    s_remap[0][0] = 1.0f;
    s_remap[1][1] = 1.0f;
    s_remap[2][2] = 1.0f;

    initQuaternionGRV();

    s_prev_yaw     = 0.0f;
    s_prev_pitch   = 0.0f;
    s_smooth_yaw   = 0.0f;
    s_smooth_pitch = 0.0f;
    s_accum_x      = 0.0f;
    s_accum_y      = 0.0f;
    s_smoothed_dt  = IMU_NOMINAL_DT;
}

void imu_mouse_init(mouse_output_cb_t output_cb) {
    s_output_cb = output_cb;
    reset_state();
    imu_add_sample_cb(on_imu_sample);
    LOG_INF("imu_mouse: initialised, collecting %d calibration samples", IMU_CAL_SAMPLES);
}

void imu_mouse_set_enabled(bool enabled) {
    bool was_enabled = s_mouse_enabled;
    s_mouse_enabled = enabled;
    if (enabled && !was_enabled) {
        reset_state(); /* fresh calibration every time we re-enter mouse mode */
        LOG_INF("imu_mouse: enabled — recalibrating");
    } else if (!enabled) {
        LOG_INF("imu_mouse: disabled");
    }
}

static bool imu_mouse_update(const struct imu_sample *s, int8_t *dx, int8_t *dy) {
    *dx = 0;
    *dy = 0;

    /* --- Compute dt from hardware ticks --- */
    float dt = compute_dt(s->ts_ticks);

    /* --- Raw counts as floats --- */
    float ax_raw = (float)s->ax;
    float ay_raw = (float)s->ay;
    float az_raw = (float)s->az;
    float gx_raw = (float)s->gx;
    float gy_raw = (float)s->gy;
    float gz_raw = (float)s->gz;

    if (!s_calibrated) {
        /* --- Calibration phase --- */
        calibration_update(ax_raw, ay_raw, az_raw,
                           gx_raw, gy_raw, gz_raw, dt);
        return false;
    }

    /* --- Post-calibration: sensor fusion → Euler → mouse delta --- */
    float yaw, pitch;
    fusion_update(ax_raw, ay_raw, az_raw,
                  gx_raw, gy_raw, gz_raw, dt,
                  &yaw, &pitch);

    return mouse_delta(yaw, pitch, dt, dx, dy);
}
