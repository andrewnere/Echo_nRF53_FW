/*
 * ble_raw_data.h — BLE GATT service that streams raw IMU samples
 *
 * Exposes a custom notify characteristic ("IMU Raw Data") carrying the
 * same timestamp + accel + gyro fields as the UART CSV output. See
 * ble_raw_data.c for the wire format and UUIDs.
 */

#ifndef BLE_RAW_DATA_H
#define BLE_RAW_DATA_H

#include <stdbool.h>

/*
 * Subscribes to imu.c's sample callback so every IMU sample is forwarded
 * as a BLE notification. Call once after bt_enable() / settings_load().
 */
void ble_raw_data_init(void);

/*
 * Enable or disable BLE raw data notifications at runtime.
 * When disabled, on_imu_sample() returns immediately (no BLE TX).
 * Enabled state is independent of whether a client has subscribed.
 */
void ble_raw_data_set_enabled(bool enabled);

#endif /* BLE_RAW_DATA_H */
