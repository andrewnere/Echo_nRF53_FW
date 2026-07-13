/*
 * app_config.h — Project-wide tunable parameters
 *
 * Single place to change IMU sample rate, calibration timing, Madgwick
 * filter aggressiveness, mouse feel (sensitivity / deadzone / smoothing),
 * and BLE report rate.  No other source files need touching.
 *
 * Rate index ↔ Hz mapping (k_rates[] in imu.c):
 *   IMU_RATE_IDX  0 →  12.5 Hz
 *   IMU_RATE_IDX  1 →  26   Hz
 *   IMU_RATE_IDX  2 →  52   Hz
 *   IMU_RATE_IDX  3 → 104   Hz
 *   IMU_RATE_IDX  4 → 208   Hz
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H



/* ============================================================
 * Compile-time feature guards.
 * Comment out a define to strip that feature from the build entirely.
 * If neither is defined the BLE stack is not initialised at all.
 *
 *   ENABLE_BLE_HID       — HID mouse profile (identity 0, "Virtual Mouse")
 *                          macOS and Windows pair with this as a standard mouse.
 *   ENABLE_BLE_RAW_DATA  — Raw IMU GATT notify service (identity 1, "VM-Raw")
 *                          Python / bleak script on Mac or Windows connects here.
 *   ENABLE_NRF53_AS_CENTRAL — nRF53 also acts as a BLE Central, connecting out
 *                          to an ESP32 peripheral and mapping its 4 ASCII
 *                          commands ("cmd1".."cmd4") to LED1-LED4. Disables
 *                          the heartbeat blink (frees led0 for cmd1).
 * ============================================================ */
#define ENABLE_BLE_HID
#define ENABLE_BLE_RAW_DATA
#define ENABLE_NRF53_AS_CENTRAL

/* ============================================================
 * Output mode — sets the active mode at boot.
 * Can be changed at runtime with the UART command "MODE:N".
 *
 *   0 — UART  : IMU → CSV on UART. No BLE activity.
 *   1 — HID   : BLE HID mouse only.          Requires ENABLE_BLE_HID.
 *   2 — Raw   : BLE raw IMU stream only.      Requires ENABLE_BLE_RAW_DATA.
 *   3 — Both  : HID mouse + raw IMU stream.   Requires both.
 * ============================================================ */
#define DEFAULT_OUTPUT_MODE  3

/* ============================================================
 * ENABLE_UART_DEBUGGING — compile-time only (independent of mode).
 * Enables [cal]/[gyro]/[imu] prints, FIFO overrun warnings, and
 * rate/mode status echoes.  Comment out for a quiet build.
 * ============================================================ */
//#define ENABLE_UART_DEBUGGING

/* ============================================================
 * Board mounting orientation.
 * Define one to hardcode the sensor→world remap and skip auto-detection.
 * Leave all commented out to use calibration-derived auto-detection.
 *
 *   IMU_MOUNT_Z_UP    — board flat, sensor chip face-up (auto-detect default)
 *   IMU_MOUNT_Y_DOWN  — board rotated 90° around X-axis, Y-axis pointing down
 * ============================================================ */
//#define IMU_MOUNT_Z_UP
#define IMU_MOUNT_Y_DOWN

/* ============================================================
 * IMU sample rate — set IMU_RATE_IDX only; IMU_ODR_HZ is derived.
 * ============================================================ */
#define IMU_RATE_IDX   2

/* Derived from IMU_RATE_IDX — do not edit */
#if   IMU_RATE_IDX == 0
#  define IMU_ODR_HZ   13    /* 12.5 Hz hardware rate; rounded for integer math */
#elif IMU_RATE_IDX == 1
#  define IMU_ODR_HZ   26
#elif IMU_RATE_IDX == 2
#  define IMU_ODR_HZ   52
#elif IMU_RATE_IDX == 3
#  define IMU_ODR_HZ   104
#elif IMU_RATE_IDX == 4
#  define IMU_ODR_HZ   208
#else
#  error "IMU_RATE_IDX must be 0–4"
#endif

/* ============================================================
 * Hardware constants — do not change without updating sensor config
 * ============================================================ */
#define IMU_GYRO_SCALE       57.143f   /* ±500 dps: 17.5 mdps/LSB → 1 dps = 57.14 LSB */
#define IMU_ACCEL_SCALE      16384.0f  /* ±2 g: 0.061 mg/LSB */
#define IMU_TS_UNIT_S        25e-6f    /* seconds per hardware timestamp tick */
#define IMU_MIN_GRAVITY_LSB  8000.0f   /* reject calibration if |accel| < this in LSB */

/* ============================================================
 * Calibration timing
 * Phase 1 (settle): discard samples while sensor output stabilises.
 * Phase 2 (collect): accumulate samples to compute gyro bias + gravity.
 * ============================================================ */
#define IMU_CAL_SETTLE_S     3
#define IMU_CAL_COLLECT_S    3
#define IMU_CAL_SKIP_SAMPLES (IMU_ODR_HZ * IMU_CAL_SETTLE_S)
#define IMU_CAL_SAMPLES      (IMU_ODR_HZ * IMU_CAL_COLLECT_S)

/* ============================================================
 * Madgwick AHRS filter
 * ============================================================ */
#define IMU_MADGWICK_BETA    0.04f   /* larger = faster convergence, more accel noise */
#define IMU_ACCEL_LPF_ALPHA  0.2f   /* EMA weight for new accel sample (0–1) */

/* ============================================================
 * dt (inter-sample period) derivation
 * ============================================================ */
#define IMU_NOMINAL_DT       (1.0f / (float)IMU_ODR_HZ)
#define IMU_MIN_DT           0.005f  /* clamp outliers below this (seconds) */
#define IMU_MAX_DT           0.1f    /* clamp outliers above this (seconds) */
#define IMU_DT_ALPHA_NEW     0.2f    /* EMA weight for newly measured dt */
#define IMU_DT_ALPHA_OLD     0.8f    /* EMA weight for previous smoothed dt */

/* ============================================================
 * Mouse feel — adjust these without reflashing algorithm code
 * ============================================================ */
#define MOUSE_SENSITIVITY_X  50.0f   /* pixels per degree of yaw */
#define MOUSE_SENSITIVITY_Y  50.0f   /* pixels per degree of pitch */
#define MOUSE_DEADZONE_X     0.1f    /* degrees — motion below this is ignored */
#define MOUSE_DEADZONE_Y     0.1f    /* degrees */
#define MOUSE_SMOOTH_TAU     0.06f   /* EMA time constant for output smoothing (seconds) */

/* ============================================================
 * BLE report rate limiter
 * Send one HID report every MOUSE_SEND_EVERY_N IMU samples.
 * Effective BLE rate = IMU_ODR_HZ / MOUSE_SEND_EVERY_N.
 *
 *   1 → full rate (IMU_ODR_HZ)
 *   2 → half rate (IMU_ODR_HZ / 2)
 *   N → IMU_ODR_HZ / N
 *
 * Motion between sends is accumulated, so no data is lost.
 * ============================================================ */
#define MOUSE_SEND_EVERY_N   1

/* Batch N IMU samples into one BLE notification.
 * Payload = N × 16 bytes; requires ATT MTU > (N×16 + 3).
 * CONFIG_BT_L2CAP_TX_MTU=100 supports up to 6 samples (96 bytes).
 *   3 → ~17 notify/sec at 52 Hz  (current config)
 */
#define BLE_RAW_BATCH_SIZE  1

#endif /* APP_CONFIG_H */
