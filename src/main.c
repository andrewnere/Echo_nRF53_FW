/*
 * main.c — BLE HID mouse + optional raw IMU data streaming
 *
 * Runtime modes (DEFAULT_OUTPUT_MODE in app_config.h, or "MODE:N" on UART):
 *   0 — UART  : IMU → CSV on UART. No BLE activity.
 *   1 — HID   : BLE HID mouse only.          Requires ENABLE_BLE_HID.
 *   2 — Raw   : BLE raw IMU stream only.      Requires ENABLE_BLE_RAW_DATA.
 *   3 — Both  : HID mouse + raw IMU stream.   Requires both.
 *
 * Compile-time guards (app_config.h):
 *   #define ENABLE_BLE_HID       — HID mouse profile (identity 0, "Virtual Mouse")
 *   #define ENABLE_BLE_RAW_DATA  — raw IMU GATT notify service (identity 1, "VM-Raw")
 *   If neither is defined the BLE stack is not initialised at all.
 */

#include "app_config.h"   /* defines ENABLE_BLE_HID / ENABLE_BLE_RAW_DATA first */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#ifdef ENABLE_BLE_HID
#include <bluetooth/services/hids.h>
#endif

#include "ble_central.h"
#include "ble_raw_data.h"
#include "imu.h"
#include "imu_mouse.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ============================================================
 * LED — 3-second blink sanity check
 * Disabled under ENABLE_NRF53_AS_CENTRAL: led0 is repurposed as LED1,
 * driven by incoming ESP32 commands instead (see ble_central.c).
 * ============================================================ */

#ifndef ENABLE_NRF53_AS_CENTRAL

#define LED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

static void led_toggle_cb(struct k_timer *timer) {
    ARG_UNUSED(timer);
    gpio_pin_toggle_dt(&led);
}
K_TIMER_DEFINE(led_timer, led_toggle_cb, NULL);

#endif /* !ENABLE_NRF53_AS_CENTRAL */

/* ============================================================
 * Mode tracking
 * ============================================================ */

static int  s_current_mode = DEFAULT_OUTPUT_MODE;
static bool s_adv_ready;   /* true once setup_advertising() has created the handles */

/* ============================================================
 * BLE HID mouse (compiled only when ENABLE_BLE_HID is defined)
 * ============================================================ */

#ifdef ENABLE_BLE_HID

#define MOUSE_REP_SIZE 3
#define MOUSE_REP_IDX  0

BT_HIDS_DEF(hids_obj, MOUSE_REP_SIZE);

static struct bt_conn *current_conn;

/*
 * HID report descriptor — 3-byte relative mouse.
 * Byte 0: buttons (3 bits) + padding (5 bits)
 * Byte 1: X movement, signed, -127 to +127
 * Byte 2: Y movement, signed, -127 to +127
 */
static const uint8_t report_map[] = {
    0x05, 0x01, /* Usage Page (Generic Desktop)       */
    0x09, 0x02, /* Usage (Mouse)                      */
    0xA1, 0x01, /* Collection (Application)           */
    0x09, 0x01, /*   Usage (Pointer)                  */
    0xA1, 0x00, /*   Collection (Physical)            */
    0x05, 0x09, /*     Usage Page (Buttons)           */
    0x19, 0x01, /*     Usage Minimum (1)              */
    0x29, 0x03, /*     Usage Maximum (3)              */
    0x15, 0x00, /*     Logical Minimum (0)            */
    0x25, 0x01, /*     Logical Maximum (1)            */
    0x75, 0x01, /*     Report Size (1 bit)            */
    0x95, 0x03, /*     Report Count (3 buttons)       */
    0x81, 0x02, /*     Input (Data, Variable, Abs)    */
    0x75, 0x05, /*     Report Size (5 bits padding)   */
    0x95, 0x01, /*     Report Count (1)               */
    0x81, 0x01, /*     Input (Constant)               */
    0x05, 0x01, /*     Usage Page (Generic Desktop)   */
    0x09, 0x30, /*     Usage (X)                      */
    0x09, 0x31, /*     Usage (Y)                      */
    0x15, 0x81, /*     Logical Minimum (-127)         */
    0x25, 0x7F, /*     Logical Maximum (+127)         */
    0x75, 0x08, /*     Report Size (8 bits)           */
    0x95, 0x02, /*     Report Count (2 axes)          */
    0x81, 0x06, /*     Input (Data, Variable, Rel)    */
    0xC0,       /*   End Collection                   */
    0xC0,       /* End Collection                     */
};

static const struct bt_data ad_hid[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL)),
};
static const struct bt_data sd_hid[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static struct bt_le_ext_adv *s_adv_hid;

static void start_hid_adv(void)
{
    int err = bt_le_ext_adv_start(s_adv_hid, BT_LE_EXT_ADV_START_DEFAULT);
    if (err == 0)              { printk("HID_ADV: started\n"); }
    else if (err != -EALREADY) { printk("HID_ADV: failed %d\n", err); }
}

static void hid_adv_work_fn(struct k_work *work) { ARG_UNUSED(work); start_hid_adv(); }
K_WORK_DEFINE(s_hid_adv_work, hid_adv_work_fn);

static const struct bt_le_conn_param k_hid_conn_params = {
    .interval_min = 6,    /* 7.5 ms */
    .interval_max = 9,    /* 11.25 ms */
    .latency      = 0,
    .timeout      = 400,  /* 4 s */
};

static void send_mouse(int8_t x, int8_t y)
{
    if (!current_conn) { return; }
    uint8_t rep[MOUSE_REP_SIZE] = {0, (uint8_t)x, (uint8_t)y};
    bt_hids_inp_rep_send(&hids_obj, current_conn, MOUSE_REP_IDX,
                         rep, sizeof(rep), NULL);
}

static void hids_init(void)
{
    struct bt_hids_init_param hids_init = {0};
    struct bt_hids_inp_rep   *inp_rep;

    hids_init.rep_map.data = report_map;
    hids_init.rep_map.size = sizeof(report_map);
    hids_init.info.bcd_hid = 0x0101;
    hids_init.info.b_country_code = 0x00;
    hids_init.info.flags = BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE;

    inp_rep = &hids_init.inp_rep_group_init.reports[0];
    inp_rep->size = MOUSE_REP_SIZE;
    inp_rep->id   = 0;
    hids_init.inp_rep_group_init.cnt = 1;
    hids_init.is_mouse = true;

    bt_hids_init(&hids_obj, &hids_init);
}

#endif /* ENABLE_BLE_HID */

/* ============================================================
 * BLE raw IMU stream (compiled only when ENABLE_BLE_RAW_DATA is defined)
 * ============================================================ */

#ifdef ENABLE_BLE_RAW_DATA

#define RAW_DEVICE_NAME "VM-Raw"
static const struct bt_data ad_raw[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, RAW_DEVICE_NAME, sizeof(RAW_DEVICE_NAME) - 1),
};

static struct bt_le_ext_adv *s_adv_raw;
static struct bt_conn       *s_raw_conn;

static void start_raw_adv(void)
{
    int err = bt_le_ext_adv_start(s_adv_raw, BT_LE_EXT_ADV_START_DEFAULT);
    if (err == 0)              { printk("RAW_ADV: started\n"); }
    else if (err != -EALREADY) { printk("RAW_ADV: failed %d\n", err); }
}

static void raw_adv_work_fn(struct k_work *work) { ARG_UNUSED(work); start_raw_adv(); }
K_WORK_DEFINE(s_raw_adv_work, raw_adv_work_fn);

static struct k_work_delayable s_raw_param_work;

/* 15–25 ms: smooth IMU streaming while leaving bandwidth for HID. */
static const struct bt_le_conn_param k_raw_conn_params = {
    .interval_min = 12,   /* 15 ms */
    .interval_max = 20,   /* 25 ms */
    .latency      = 0,
    .timeout      = 400,  /* 4 s */
};

static void raw_param_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);
    if (s_raw_conn) {
        bt_conn_le_param_update(s_raw_conn, &k_raw_conn_params);
    }
}

#endif /* ENABLE_BLE_RAW_DATA */

/* ============================================================
 * Advertising — mode-aware start / stop
 * ============================================================ */

static void apply_adv_for_mode(int mode)
{
#ifdef ENABLE_BLE_HID
    if (s_adv_hid) { bt_le_ext_adv_stop(s_adv_hid); }
    if (s_adv_hid && (mode == 1 || mode == 3)) { start_hid_adv(); }
#endif
#ifdef ENABLE_BLE_RAW_DATA
    if (s_adv_raw) { bt_le_ext_adv_stop(s_adv_raw); }
    if (s_adv_raw && (mode == 2 || mode == 3)) { start_raw_adv(); }
#endif
}

/* ============================================================
 * Connection callbacks
 * ============================================================ */

static void security_changed(struct bt_conn *conn, bt_security_t level,
                              enum bt_security_err err)
{
    if (err) { printk("SECURITY_FAILED: err=%d\n", err); return; }
    printk("SECURITY_OK: level=%d\n", level);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { return; }
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    printk("CONNECTED: id=%u interval=%u (%.2f ms)\n",
           info.id, info.le.interval, (float)info.le.interval * 1.25f);

#ifdef ENABLE_BLE_HID
    if (info.id == BT_ID_DEFAULT && info.role == BT_CONN_ROLE_PERIPHERAL) {
        bt_hids_connected(&hids_obj, conn);
        current_conn = bt_conn_ref(conn);
        bt_conn_set_security(conn, BT_SECURITY_L2);
        bt_conn_le_param_update(conn, &k_hid_conn_params);
    }
#endif

#ifdef ENABLE_BLE_RAW_DATA
    if (info.id == 1) {
        if (s_raw_conn) { bt_conn_unref(s_raw_conn); }
        s_raw_conn = bt_conn_ref(conn);
        k_work_schedule(&s_raw_param_work, K_MSEC(500));
    }
#endif
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    printk("DISCONNECTED: id=%u reason=%u\n", info.id, reason);

#ifdef ENABLE_BLE_HID
    if (info.id == BT_ID_DEFAULT && info.role == BT_CONN_ROLE_PERIPHERAL) {
        bt_hids_disconnected(&hids_obj, conn);
        bt_conn_unref(current_conn);
        current_conn = NULL;
        if (s_current_mode == 1 || s_current_mode == 3) {
            k_work_submit(&s_hid_adv_work);
        }
        return;
    }
#endif

#ifdef ENABLE_BLE_RAW_DATA
    if (info.id == 1) {
        k_work_cancel_delayable(&s_raw_param_work);
        bt_conn_unref(s_raw_conn);
        s_raw_conn = NULL;
        if (s_current_mode == 2 || s_current_mode == 3) {
            k_work_submit(&s_raw_adv_work);
        }
    }
#endif
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                              uint16_t latency, uint16_t timeout)
{
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    printk("CONN_PARAMS:id=%u interval=%u (%.2f ms) latency=%u timeout=%u\n",
           info.id, interval, (float)interval * 1.25f, latency, timeout);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected        = connected,
    .disconnected     = disconnected,
    .security_changed = security_changed,
    .le_param_updated = le_param_updated,
};

static void auth_cancel(struct bt_conn *conn) { ARG_UNUSED(conn); }
static struct bt_conn_auth_cb auth_cb = { .cancel = auth_cancel };

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    ARG_UNUSED(reason);
    bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}
static struct bt_conn_auth_info_cb auth_info_cb = { .pairing_failed = pairing_failed };

/* ============================================================
 * Output mode control
 * ============================================================ */

static void apply_output_mode(int mode)
{
    s_current_mode = mode;

    imu_set_uart_raw(mode == 0);

#ifdef ENABLE_BLE_HID
    imu_mouse_set_enabled(mode == 1 || mode == 3);
#endif
#ifdef ENABLE_BLE_RAW_DATA
    ble_raw_data_set_enabled(mode == 2 || mode == 3);
#endif

    if (s_adv_ready) {
        apply_adv_for_mode(mode);
    }
}

/* ============================================================
 * Advertising setup
 * Creates adv set handles; apply_adv_for_mode() decides which to start.
 * ============================================================ */

static void setup_advertising(void)
{
    int err;

#ifdef ENABLE_BLE_HID
    {
        struct bt_le_adv_param p = {
            .id                = BT_ID_DEFAULT,
            .sid               = 0,
            .secondary_max_skip = 0,
            .options           = BT_LE_ADV_OPT_CONNECTABLE,
            .interval_min      = BT_GAP_ADV_FAST_INT_MIN_2,
            .interval_max      = BT_GAP_ADV_FAST_INT_MAX_2,
            .peer              = NULL,
        };
        err = bt_le_ext_adv_create(&p, NULL, &s_adv_hid);
        if (err) { printk("HID_ADV_CREATE_ERR: %d\n", err); return; }
        bt_le_ext_adv_set_data(s_adv_hid,
                               ad_hid, ARRAY_SIZE(ad_hid),
                               sd_hid, ARRAY_SIZE(sd_hid));
    }
#endif

#ifdef ENABLE_BLE_RAW_DATA
    {
        struct bt_le_adv_param p = {
            .id                = 1,
            .sid               = 0,
            .secondary_max_skip = 0,
            .options           = BT_LE_ADV_OPT_CONNECTABLE,
            .interval_min      = BT_GAP_ADV_FAST_INT_MIN_2,
            .interval_max      = BT_GAP_ADV_FAST_INT_MAX_2,
            .peer              = NULL,
        };
        err = bt_le_ext_adv_create(&p, NULL, &s_adv_raw);
        if (err) { printk("RAW_ADV_CREATE_ERR: %d\n", err); return; }
        bt_le_ext_adv_set_data(s_adv_raw, ad_raw, ARRAY_SIZE(ad_raw), NULL, 0);
    }
#endif

    s_adv_ready = true;
    apply_adv_for_mode(s_current_mode);
}

/* ============================================================
 * Entry point
 * ============================================================ */

int main(void)
{
#ifndef ENABLE_NRF53_AS_CENTRAL
    int ret;

    if (!gpio_is_ready_dt(&led)) { return -1; }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) { return ret; }
    k_timer_start(&led_timer, K_SECONDS(3), K_SECONDS(3));
#endif

    imu_set_mode_change_cb(apply_output_mode);

#if defined(ENABLE_BLE_HID) || defined(ENABLE_BLE_RAW_DATA) || defined(ENABLE_NRF53_AS_CENTRAL)
    bt_conn_auth_cb_register(&auth_cb);
    bt_conn_auth_info_cb_register(&auth_info_cb);
    bt_enable(NULL);
    settings_load();

#ifdef ENABLE_BLE_RAW_DATA
    /* Identity 1 ("VM-Raw") — created once, address persisted to NVS.
     * Returns -ENOMEM when already at CONFIG_BT_ID_MAX after settings_load(). */
    {
        int id = bt_id_create(NULL, NULL);
        if (id < 0 && id != -ENOMEM) { printk("ID_CREATE_ERR: %d\n", id); }
    }
#endif

#ifdef ENABLE_BLE_HID
    hids_init();
    imu_mouse_init(send_mouse);
#endif
#ifdef ENABLE_BLE_RAW_DATA
    ble_raw_data_init();
    k_work_init_delayable(&s_raw_param_work, raw_param_work_fn);
#endif

    apply_output_mode(DEFAULT_OUTPUT_MODE);
    setup_advertising();

#ifdef ENABLE_NRF53_AS_CENTRAL
    ble_central_init();
#endif

#else  /* no BLE features compiled in — UART-only build */
    apply_output_mode(DEFAULT_OUTPUT_MODE);
#endif

    return 0;
}
