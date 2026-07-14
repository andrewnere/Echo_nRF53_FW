/*
 * imu_mouse.h — IMU-to-mouse-delta processor
 *
 * Two-stage pipeline:
 *   Stage 1 — Sensor fusion: gyro bias calibration + mounting detection +
 *             updateQuaternionGRV (Game Rotation Vector: gradient-descent
 *             accel+gyro fusion) → Euler angles (yaw, pitch). Internal
 *             variable names in imu_mouse.c follow the GRV section of the
 *             author's own fusion.c (SensorFusion/) for cross-reference.
 *   Stage 2 — Mouse delta: rate-independent EMA smoothing → angular delta
 *             → deadzone gate → sensitivity → int8 HID output with sub-pixel
 *             accumulation.
 *
 * Usage:
 *   imu_mouse_init(send_mouse);   // call once after bt_enable(); self-registers callback
 */

#ifndef IMU_MOUSE_H
#define IMU_MOUSE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Output callback type: called with (dx, dy) whenever motion is detected.
 * Signature matches send_mouse() in main.c so it can be passed directly.
 */
typedef void (*mouse_output_cb_t)(int8_t dx, int8_t dy);

/*
 * Initialise all internal state and register output_cb as the mouse report
 * sink.  Must be called once; self-registers with imu_add_sample_cb().
 */
void imu_mouse_init(mouse_output_cb_t output_cb);

/*
 * Enable or disable the mouse pipeline at runtime.
 *
 * Disabling stops GRV processing and suppresses HID reports immediately.
 * Re-enabling triggers a fresh calibration cycle (settle + collect), so the
 * cursor is ready once calibration completes (~6 s at default settings).
 */
void imu_mouse_set_enabled(bool enabled);

#endif /* IMU_MOUSE_H */
