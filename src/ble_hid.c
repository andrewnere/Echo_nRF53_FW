/*
 * ble_hid.c — BLE HID mouse profile (identity 0, "Virtual Mouse")
 */

#include <stdbool.h>

#include "app_config.h"

#ifdef ENABLE_BLE_HID

#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "ble_hid.h"
#include "imu_mouse.h"

LOG_MODULE_REGISTER(ble_hid, LOG_LEVEL_INF);

#define MOUSE_REP_SIZE 3
#define MOUSE_REP_IDX  0

BT_HIDS_DEF(hids_obj, MOUSE_REP_SIZE);

static struct bt_conn *current_conn;
static bool            s_active;

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

/* ============================================================
 * Connection callbacks — filtered to this identity only (id 0, peripheral
 * role); the raw-IMU / central links are handled by their own modules.
 * ============================================================ */

static void hid_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { return; }

    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    if (info.id != BT_ID_DEFAULT || info.role != BT_CONN_ROLE_PERIPHERAL) {
        return;
    }

    bt_hids_connected(&hids_obj, conn);
    current_conn = bt_conn_ref(conn);
    bt_conn_set_security(conn, BT_SECURITY_L2);
    bt_conn_le_param_update(conn, &k_hid_conn_params);
}

static void hid_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(reason);

    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    if (info.id != BT_ID_DEFAULT || info.role != BT_CONN_ROLE_PERIPHERAL) {
        return;
    }

    bt_hids_disconnected(&hids_obj, conn);
    bt_conn_unref(current_conn);
    current_conn = NULL;

    if (s_active) {
        k_work_submit(&s_hid_adv_work);
    }
}

BT_CONN_CB_DEFINE(hid_conn_callbacks) = {
    .connected    = hid_connected,
    .disconnected = hid_disconnected,
};

/* ============================================================
 * Public API
 * ============================================================ */

void ble_hid_init(void)
{
    struct bt_le_adv_param p = {
        .id                 = BT_ID_DEFAULT,
        .sid                = 0,
        .secondary_max_skip = 0,
        .options            = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min       = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max       = BT_GAP_ADV_FAST_INT_MAX_2,
        .peer               = NULL,
    };
    int err = bt_le_ext_adv_create(&p, NULL, &s_adv_hid);
    if (err) { printk("HID_ADV_CREATE_ERR: %d\n", err); return; }
    bt_le_ext_adv_set_data(s_adv_hid, ad_hid, ARRAY_SIZE(ad_hid), sd_hid, ARRAY_SIZE(sd_hid));

    hids_init();
    imu_mouse_init(send_mouse);
}

void ble_hid_set_active(bool active)
{
    s_active = active;
    imu_mouse_set_enabled(active);

    if (s_adv_hid) { bt_le_ext_adv_stop(s_adv_hid); }
    if (s_adv_hid && active) { start_hid_adv(); }
}

#else /* ENABLE_BLE_HID not defined — provide no-op stubs */

void ble_hid_init(void)             {}
void ble_hid_set_active(bool a)     { (void)a; }

#endif /* ENABLE_BLE_HID */
