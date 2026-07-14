/*
 * imu_mouse.c — IMU-to-mouse-delta processor for LSM6DSO on nRF5340
 *
 * Ported from the Python IMU processor algorithm.  Two stages:
 *
 * Stage 1 — Sensor Fusion
 *   - Collect IMU_CAL_SAMPLES samples while stationary to derive:
 *       gyro_bias[3]      : mean raw gyro counts (subtract before integration)
 *       gravity_vec[3]    : mean raw accel counts (establishes world-up axis)
 *   - Detect mounting orientation from dominant gravity axis; build 3×3 remap
 *     matrix that maps sensor frame → world frame.
 *   - Run Madgwick AHRS throughout calibration so the filter converges before
 *     we start emitting mouse deltas.
 *   - Post-calibration: apply bias + remap, run LPF + Madgwick → Euler.
 *
 * Stage 2 — Mouse Delta
 *   - Rate-independent EMA smoothing on yaw + pitch.
 *   - Angular delta gated by per-axis deadzone.
 *   - Scale by per-axis sensitivity → float dx/dy.
 *   - Sub-pixel accumulation → int8 HID report values.
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

/* ---------- Madgwick quaternion [w, x, y, z] ---------- */
static float    s_q[4];

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

/* ============================================================
 * Mounting detection + remap matrix
 *
 * Given a normalised gravity vector gnorm[], finds the dominant axis and
 * fills s_remap[3][3] with the appropriate permutation/sign matrix so that
 * the remapped accel points in the +Z direction in world frame.
 * ============================================================ */
static void build_remap_matrix(const float gnorm[3]) {
    /* Find dominant axis: 0=X, 1=Y, 2=Z */
    int   dom = 0;
    float best = fabsf(gnorm[0]);

    for (int i = 1; i < 3; i++) {
        float a = fabsf(gnorm[i]);
        if (a > best) {
            best = a;
            dom = i;
        }
    }

    /* Zero the matrix then fill the correct case */
    memset(s_remap, 0, sizeof(s_remap));

    if (dom == 2) {
        if (gnorm[2] > 0.0f) {
            /* Z-up: identity */
            s_remap[0][0] = 1.0f;
            s_remap[1][1] = 1.0f;
            s_remap[2][2] = 1.0f;
            LOG_INF("Mounting: Z-up (identity)");
        } else {
            /* Z-down: diag(1,-1,-1) */
            s_remap[0][0] =  1.0f;
            s_remap[1][1] = -1.0f;
            s_remap[2][2] = -1.0f;
            LOG_INF("Mounting: Z-down");
        }
    } else if (dom == 1) {
        if (gnorm[1] > 0.0f) {
            /* Y-up: [[1,0,0],[0,0,1],[0,-1,0]] */
            s_remap[0][0] =  1.0f;
            s_remap[1][2] =  1.0f;
            s_remap[2][1] = -1.0f;
            LOG_INF("Mounting: Y-up");
        } else {
            /* Y-down: [[1,0,0],[0,0,-1],[0,1,0]] */
            s_remap[0][0] =  1.0f;
            s_remap[1][2] = -1.0f;
            s_remap[2][1] =  1.0f;
            LOG_INF("Mounting: Y-down");
        }
    } else { /* dom == 0 */
        if (gnorm[0] > 0.0f) {
            /* X-up: [[0,0,-1],[0,1,0],[1,0,0]] */
            s_remap[0][2] = -1.0f;
            s_remap[1][1] =  1.0f;
            s_remap[2][0] =  1.0f;
            LOG_INF("Mounting: X-up");
        } else {
            /* X-down: [[0,0,1],[0,1,0],[-1,0,0]] */
            s_remap[0][2] =  1.0f;
            s_remap[1][1] =  1.0f;
            s_remap[2][0] = -1.0f;
            LOG_INF("Mounting: X-down");
        }
    }
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
 * Madgwick AHRS update
 *
 * Ported verbatim from the Python MadgwickAHRS.update() method.
 *
 * Inputs:
 *   gx_dps, gy_dps, gz_dps — gyro in degrees per second
 *   ax_g,   ay_g,   az_g   — accel in g (already divided by IMU_ACCEL_SCALE)
 *   dt                     — integration step in seconds
 *
 * Updates s_q[] in-place.  Quaternion convention: [w, x, y, z] = [q0..q3].
 * ============================================================ */
static void madgwick_update(float gx_dps, float gy_dps, float gz_dps,
                            float ax_g,   float ay_g,   float az_g,
                            float dt) {
    float q0 = s_q[0], q1 = s_q[1], q2 = s_q[2], q3 = s_q[3];

    /* --- Normalise accelerometer --- */
    float anorm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    if (anorm < 1e-10f) {
        return; /* free-fall or bad data — skip this update */
    }
    float ax = ax_g / anorm;
    float ay = ay_g / anorm;
    float az = az_g / anorm;

    /* --- Gradient descent objective function (f) --- */
    float f0 = 2.0f*(q1*q3 - q0*q2) - ax;
    float f1 = 2.0f*(q0*q1 + q2*q3) - ay;
    float f2 = 1.0f - 2.0f*(q1*q1 + q2*q2) - az;

    /* --- Jacobian transpose times f (s = J^T · f) --- */
    float s0 = -2.0f*q2*f0 + 2.0f*q1*f1;
    float s1 =  2.0f*q3*f0 + 2.0f*q0*f1 - 4.0f*q1*f2;
    float s2 = -2.0f*q0*f0 + 2.0f*q3*f1 - 4.0f*q2*f2;
    float s3 =  2.0f*q1*f0 + 2.0f*q2*f1;

    /* --- Normalise step --- */
    float snorm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (snorm > 1e-10f) {
        s0 /= snorm;
        s1 /= snorm;
        s2 /= snorm;
        s3 /= snorm;
    }

    /* --- Gyro to radians per second --- */
    float gr  = gx_dps * DEG2RAD;
    float gpr = gy_dps * DEG2RAD;
    float gzr = gz_dps * DEG2RAD;

    /* --- Quaternion rate of change --- */
    float beta = IMU_MADGWICK_BETA;
    float qd0 = 0.5f*(-q1*gr  - q2*gpr - q3*gzr) - beta*s0;
    float qd1 = 0.5f*( q0*gr  + q2*gzr - q3*gpr) - beta*s1;
    float qd2 = 0.5f*( q0*gpr - q1*gzr + q3*gr ) - beta*s2;
    float qd3 = 0.5f*( q0*gzr + q1*gpr - q2*gr ) - beta*s3;

    /* --- Integrate --- */
    q0 += qd0 * dt;
    q1 += qd1 * dt;
    q2 += qd2 * dt;
    q3 += qd3 * dt;

    /* --- Normalise quaternion --- */
    float qnorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (qnorm < 1e-10f) {
        /* Degenerate — reset to identity */
        s_q[0] = 1.0f; s_q[1] = 0.0f; s_q[2] = 0.0f; s_q[3] = 0.0f;
        return;
    }
    q0 /= qnorm;
    q1 /= qnorm;
    q2 /= qnorm;
    q3 /= qnorm;

    s_q[0] = q0; s_q[1] = q1; s_q[2] = q2; s_q[3] = q3;
}

/* ============================================================
 * Euler angles from quaternion (degrees)
 *
 * Fills *roll, *pitch, *yaw from the current s_q[] state.
 * ============================================================ */
static void quat_to_euler(float *roll, float *pitch, float *yaw) {
    float q0 = s_q[0], q1 = s_q[1], q2 = s_q[2], q3 = s_q[3];

    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;

    float sinp = 2.0f*(q0*q2 - q3*q1);
    /* clamp to [-1, 1] before asin to avoid NaN at ±90° */
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * RAD2DEG;

    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;
}

/* ============================================================
 * Calibration phase
 *
 * Called for each of the first IMU_CAL_SAMPLES samples.  Accumulates gyro and
 * accel sums, runs the Madgwick filter (so it converges during the idle
 * calibration window), and on the last sample:
 *   1. Computes gyro bias and gravity vector from the means.
 *   2. Detects mounting and builds the remap matrix.
 *   3. Sets s_calibrated = true.
 *
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
        /* Run Madgwick during skip so it converges before collection ends */
        float gx_dps = gx_raw / IMU_GYRO_SCALE;
        float gy_dps = gy_raw / IMU_GYRO_SCALE;
        float gz_dps = gz_raw / IMU_GYRO_SCALE;
        madgwick_update(gx_dps, gy_dps, gz_dps,
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

    /* --- Run Madgwick with filtered accel so it converges --- */
    float gx_dps = gx_raw / IMU_GYRO_SCALE;
    float gy_dps = gy_raw / IMU_GYRO_SCALE;
    float gz_dps = gz_raw / IMU_GYRO_SCALE;
    float ax_g   = s_accel_lpf[0] / IMU_ACCEL_SCALE;
    float ay_g   = s_accel_lpf[1] / IMU_ACCEL_SCALE;
    float az_g   = s_accel_lpf[2] / IMU_ACCEL_SCALE;
    madgwick_update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt);

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
    build_remap_matrix(s_gravity);
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
 * Madgwick update, then converts the quaternion to Euler angles.
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

    /* --- Madgwick update (gyro in dps, accel in g) --- */
    float gx_dps = r_vec[0] / IMU_GYRO_SCALE;
    float gy_dps = r_vec[1] / IMU_GYRO_SCALE;
    float gz_dps = r_vec[2] / IMU_GYRO_SCALE;
    float ax_g   = g_vec[0] / IMU_ACCEL_SCALE;
    float ay_g   = g_vec[1] / IMU_ACCEL_SCALE;
    float az_g   = g_vec[2] / IMU_ACCEL_SCALE;
    madgwick_update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt);

    /* Extract Euler angles: yaw → cursor X, pitch → cursor Y, roll discarded. */
    float roll_unused;
    quat_to_euler(&roll_unused, pitch_out, yaw_out);
    (void)roll_unused;
}

/* ============================================================
 * Mouse delta computation (Stage 2)
 *
 * Rate-independent EMA smoothing → angular delta → deadzone → sensitivity.
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
            quat_to_euler(&roll, &pitch, &yaw);
            printk("[imu] q=[%d,%d,%d,%d]/1k yaw=%d pitch=%d roll=%d dx=%d dy=%d\n",
                   (int)(s_q[0] * 1000), (int)(s_q[1] * 1000),
                   (int)(s_q[2] * 1000), (int)(s_q[3] * 1000),
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

    s_q[0] = 1.0f; s_q[1] = 0.0f; s_q[2] = 0.0f; s_q[3] = 0.0f;

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
