/* app_config.h contains all key compile time configuration parameters*/
#include "app_config.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "ble_central.h"
#include "ble_hid.h"
#include "ble_raw_data.h"
#include "imu.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ============================================================
 * Connection callbacks — generic, identity-agnostic logging only.
 * Each BLE module (ble_hid.c, ble_raw_data.c, ble_central.c) registers its
 * own BT_CONN_CB_DEFINE for identity-specific behavior; Zephyr fans out
 * connection events to every registered callback struct.
 * ============================================================ */

static void connected(struct bt_conn* conn, uint8_t err) {
    if (err) {
        return;
    }
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    DBG_PRINTK("CONNECTED: id=%u role=%u interval=%u (%.2f ms)\n",
               info.id, info.role, info.le.interval, (float)info.le.interval * 1.25f);
}

static void disconnected(struct bt_conn* conn, uint8_t reason) {
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    DBG_PRINTK("DISCONNECTED: id=%u role=%u reason=%u\n", info.id, info.role, reason);
}

static void security_changed(struct bt_conn* conn, bt_security_t level,
                             enum bt_security_err err) {
    if (err) {
        DBG_PRINTK("SECURITY_FAILED: err=%d\n", err);
        return;
    }
    DBG_PRINTK("SECURITY_OK: level=%d\n", level);
}

static void le_param_updated(struct bt_conn* conn, uint16_t interval,
                             uint16_t latency, uint16_t timeout) {
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    DBG_PRINTK("CONN_PARAMS:id=%u interval=%u (%.2f ms) latency=%u timeout=%u\n",
               info.id, interval, (float)interval * 1.25f, latency, timeout);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed,
    .le_param_updated = le_param_updated,
};

static void auth_cancel(struct bt_conn* conn) {
    ARG_UNUSED(conn);
}
static struct bt_conn_auth_cb auth_cb = {.cancel = auth_cancel};

static void pairing_failed(struct bt_conn* conn, enum bt_security_err reason) {
    ARG_UNUSED(reason);
    bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}
static struct bt_conn_auth_info_cb auth_info_cb = {.pairing_failed = pairing_failed};

/* ============================================================
 * Output mode control
 * ============================================================ */

static void apply_output_mode(int mode) {
    imu_set_uart_raw(mode == 0);

#ifdef ENABLE_BLE_HID
    ble_hid_set_active(mode == 1 || mode == 3);
#endif
#ifdef ENABLE_BLE_RAW_DATA
    ble_raw_data_set_enabled(mode == 2 || mode == 3);
#endif
}

/* ============================================================
 * Entry point
 * ============================================================ */

int main(void) {
    imu_set_mode_change_cb(apply_output_mode);

#if defined(ENABLE_BLE_HID) || defined(ENABLE_BLE_RAW_DATA) || defined(ENABLE_NRF53_AS_CENTRAL)
    bt_conn_auth_cb_register(&auth_cb);
    bt_conn_auth_info_cb_register(&auth_info_cb);
    bt_enable(NULL);
    settings_load();

#ifdef ENABLE_BLE_HID
    ble_hid_init();
#endif
#ifdef ENABLE_BLE_RAW_DATA
    ble_raw_data_init();
#endif
#ifdef ENABLE_NRF53_AS_CENTRAL
    ble_central_init();
#endif

    apply_output_mode(DEFAULT_OUTPUT_MODE);
#else /* no BLE features compiled in — UART-only build */
    apply_output_mode(DEFAULT_OUTPUT_MODE);
#endif

    return 0;
}
