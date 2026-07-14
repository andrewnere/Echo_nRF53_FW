/*
 * ble_hid.h — BLE HID mouse profile (identity 0, "Virtual Mouse")
 *
 * Advertises and exposes a standard HID-over-GATT mouse. See ble_hid.c for
 * the report descriptor and advertising data.
 */

#ifndef BLE_HID_H
#define BLE_HID_H

#include <stdbool.h>

/*
 * Creates the HID adv set, initialises the HIDS GATT service, and wires up
 * imu_mouse.c's output callback. Does not start advertising. Call once
 * after bt_enable() / settings_load().
 */
void ble_hid_init(void);

/*
 * Enable or disable the HID mouse at runtime: starts/stops advertising and
 * enables/disables imu_mouse.c's motion output.
 */
void ble_hid_set_active(bool active);

#endif /* BLE_HID_H */
