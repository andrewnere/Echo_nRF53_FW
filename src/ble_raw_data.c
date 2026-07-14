/*
 * ble_raw_data.c — "IMU Raw Data" BLE GATT notify service (optional)
 *
 * Architecture:
 *   IMU callback → ring buffer (never touches BLE)
 *   k_work_delayable drains ring → bt_gatt_notify
 *
 * Drain fires immediately once BLE_RAW_BATCH_SIZE samples are available.
 * Consecutive drains are paced at 10 ms; -ENOMEM backs off 50 ms.
 */

#include <stdbool.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble_raw_data.h"
#include "imu.h"
#include "app_config.h"

LOG_MODULE_REGISTER(ble_raw_data, LOG_LEVEL_INF);

#ifdef ENABLE_BLE_RAW_DATA

#define BT_UUID_IMU_SVC_VAL \
    BT_UUID_128_ENCODE(0xa0f0e0d0, 0x0001, 0x0000, 0x0000, 0x000000000000ULL)
#define BT_UUID_IMU_RAW_VAL \
    BT_UUID_128_ENCODE(0xa0f0e0d0, 0x0002, 0x0000, 0x0000, 0x000000000000ULL)

static struct bt_uuid_128 imu_svc_uuid = BT_UUID_INIT_128(BT_UUID_IMU_SVC_VAL);
static struct bt_uuid_128 imu_raw_uuid = BT_UUID_INIT_128(BT_UUID_IMU_RAW_VAL);

struct __packed imu_notify_payload {
    uint32_t ts_ticks;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
};

BUILD_ASSERT(sizeof(struct imu_notify_payload) == 16, "IMU notify payload must be 16 bytes");

static bool s_notify_enabled;
static bool s_enabled;
static bool s_active;   /* mirrors mode gating; drives advertising start/stop */

/* ============================================================
 * Advertising — identity 1 ("VM-Raw")
 * ============================================================ */

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

/* ============================================================
 * Connection callbacks — filtered to identity 1 only; the HID / central
 * links are handled by their own modules.
 * ============================================================ */

static void raw_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { return; }

    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    if (info.id != 1) { return; }

    if (s_raw_conn) { bt_conn_unref(s_raw_conn); }
    s_raw_conn = bt_conn_ref(conn);
    k_work_schedule(&s_raw_param_work, K_MSEC(500));
}

static void raw_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(reason);

    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    if (info.id != 1) { return; }

    k_work_cancel_delayable(&s_raw_param_work);
    bt_conn_unref(s_raw_conn);
    s_raw_conn = NULL;

    if (s_active) {
        k_work_submit(&s_raw_adv_work);
    }
}

BT_CONN_CB_DEFINE(raw_conn_callbacks) = {
    .connected    = raw_connected,
    .disconnected = raw_disconnected,
};

void ble_raw_data_set_enabled(bool enabled) {
    s_enabled = enabled;
    s_active  = enabled;
    printk("STATUS:BLE_RAW:%s\n", enabled ? "on" : "off");

    if (s_adv_raw) { bt_le_ext_adv_stop(s_adv_raw); }
    if (s_adv_raw && enabled) { start_raw_adv(); }
}

static void imu_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    s_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("STATUS:BLE_RAW_CLIENT:%s\n", s_notify_enabled ? "subscribed" : "unsubscribed");
}

BT_GATT_SERVICE_DEFINE(imu_svc,
    BT_GATT_PRIMARY_SERVICE(&imu_svc_uuid),
    BT_GATT_CHARACTERISTIC(&imu_raw_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(imu_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CUD("IMU Raw Data", BT_GATT_PERM_READ),
);

/* ---- Ring buffer ---- */

#define RING_CAP  256u   /* ~5 s at 52 Hz; power-of-2 for fast masking */
#define RING_MASK (RING_CAP - 1u)

static struct imu_notify_payload s_ring[RING_CAP];
static uint32_t s_ring_head; /* written by IMU callback only */
static uint32_t s_ring_tail; /* read/written by drain work only */

static struct k_work_delayable s_drain_work;

static void drain_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    uint32_t avail = s_ring_head - s_ring_tail;
    if (avail < BLE_RAW_BATCH_SIZE) {
        return;
    }

    struct imu_notify_payload batch[BLE_RAW_BATCH_SIZE];
    for (int i = 0; i < BLE_RAW_BATCH_SIZE; i++) {
        batch[i] = s_ring[(s_ring_tail + i) & RING_MASK];
    }

    int err = bt_gatt_notify(NULL, &imu_svc.attrs[2], batch, sizeof(batch));
    if (err == 0) {
        s_ring_tail += BLE_RAW_BATCH_SIZE;
        if ((s_ring_head - s_ring_tail) >= BLE_RAW_BATCH_SIZE) {
            k_work_schedule(&s_drain_work, K_MSEC(10));
        }
    } else if (err == -ENOMEM) {
        k_work_schedule(&s_drain_work, K_MSEC(50));
    }
}

static void on_imu_sample(const struct imu_sample *s) {
    if (!s_enabled || !s_notify_enabled) {
        return;
    }

    /* Drop oldest if ring is full */
    if ((s_ring_head - s_ring_tail) >= RING_CAP) {
        s_ring_tail++;
    }

    s_ring[s_ring_head & RING_MASK] = (struct imu_notify_payload){
        .ts_ticks = s->ts_ticks,
        .ax = s->ax, .ay = s->ay, .az = s->az,
        .gx = s->gx, .gy = s->gy, .gz = s->gz,
    };
    s_ring_head++;

    if ((s_ring_head - s_ring_tail) >= BLE_RAW_BATCH_SIZE) {
        k_work_schedule(&s_drain_work, K_NO_WAIT);
    }
}

void ble_raw_data_init(void) {
    /* Identity 1 ("VM-Raw") — created once, address persisted to NVS.
     * Returns -ENOMEM when already at CONFIG_BT_ID_MAX after settings_load(). */
    int id = bt_id_create(NULL, NULL);
    if (id < 0 && id != -ENOMEM) { printk("RAW_ID_CREATE_ERR: %d\n", id); }

    struct bt_le_adv_param p = {
        .id                 = 1,
        .sid                = 0,
        .secondary_max_skip = 0,
        .options            = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min       = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max       = BT_GAP_ADV_FAST_INT_MAX_2,
        .peer               = NULL,
    };
    int err = bt_le_ext_adv_create(&p, NULL, &s_adv_raw);
    if (err) { printk("RAW_ADV_CREATE_ERR: %d\n", err); return; }
    bt_le_ext_adv_set_data(s_adv_raw, ad_raw, ARRAY_SIZE(ad_raw), NULL, 0);

    k_work_init_delayable(&s_raw_param_work, raw_param_work_fn);
    k_work_init_delayable(&s_drain_work, drain_work_fn);
    imu_add_sample_cb(on_imu_sample);
    LOG_INF("IMU_RAW_DATA service ready");
}

#else /* ENABLE_BLE_RAW_DATA not defined — provide no-op stubs */

void ble_raw_data_init(void)              {}
void ble_raw_data_set_enabled(bool e)     { (void)e; }

#endif /* ENABLE_BLE_RAW_DATA */
