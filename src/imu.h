/*
 * imu.h — LSM6DSO IMU interface
 *
 * Two threads are defined in imu.c and start automatically:
 *   imu_tid  — drains the FIFO, prints CSV to UART, fires the sample callback
 *   cmd_tid  — reads UART for RATE:1..RATE:5 commands
 *
 * CSV output format (one line per complete accel+gyro+timestamp set):
 *   timestamp,ax,ay,az,gx,gy,gz
 *
 * All sensor values are raw int16 straight from the LSM6DSO registers.
 * Apply these scale factors in post-processing:
 *
 *   timestamp : 25 µs per tick  →  tick × 0.025 = ms,  tick × 0.000025 = s
 *   ax/ay/az  : accel raw → × 0.061  = milli-g   (±2 g range)
 *   gx/gy/gz  : gyro  raw → × 0.0175 = dps       (±500 dps range)
 *
 * UART commands (ASCII, terminated with \n or \r):
 *   RATE:1 → 12.5 Hz
 *   RATE:2 → 26   Hz
 *   RATE:3 → 52   Hz
 *   RATE:4 → 104  Hz
 *   RATE:5 → 208  Hz
 *   MODE:0 → UART raw CSV (no BLE)
 *   MODE:1 → BLE HID mouse only
 *   MODE:2 → BLE raw IMU stream only
 *   MODE:3 → BLE HID + raw IMU stream
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

/* One complete IMU sample produced by imu.c — same fields as the CSV row. */
struct imu_sample {
    uint32_t ts_ticks; /* hardware timestamp, 25 µs per tick */
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
};

/*
 * Register a callback to receive every sample as it is emitted.
 * Called from the IMU thread — keep the handler non-blocking.
 * Pass NULL to deregister.  Consumers (e.g. ble_raw_data.c) call this once
 * at init; imu.c has no knowledge of who is listening.
 */
typedef void (*imu_sample_cb_t)(const struct imu_sample* s);

/* Register a callback in the next free slot (up to 4 total).
 * Returns 0 on success, -ENOMEM if all slots are full. */
int imu_add_sample_cb(imu_sample_cb_t cb);

/* Enable or disable the UART CSV raw-data stream (MODE:1). */
void imu_set_uart_raw(bool enabled);

/*
 * Register a callback invoked whenever a MODE:N command arrives.
 * The callback receives the new mode integer (1–4).  Called from the
 * cmd thread — the handler must be non-blocking.
 */
void imu_set_mode_change_cb(void (*cb)(int mode));

#endif /* IMU_H */
