/*
 * ble_central.h — nRF53 acting as a BLE Central, connected to an ESP32
 * peripheral. See ble_central.c for the service/characteristic UUIDs,
 * command set, and LED mapping.
 */

#ifndef BLE_CENTRAL_H
#define BLE_CENTRAL_H

/*
 * Configures the command LEDs and starts scanning for the ESP32 peripheral.
 * Call once after bt_enable() / settings_load().
 */
void ble_central_init(void);

#endif /* BLE_CENTRAL_H */
