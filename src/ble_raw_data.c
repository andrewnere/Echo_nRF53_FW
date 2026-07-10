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

void ble_raw_data_set_enabled(bool enabled) {
    s_enabled = enabled;
    printk("STATUS:BLE_RAW:%s\n", enabled ? "on" : "off");
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
    k_work_init_delayable(&s_drain_work, drain_work_fn);
    imu_add_sample_cb(on_imu_sample);
    LOG_INF("IMU_RAW_DATA service ready");
}

#else /* ENABLE_BLE_RAW_DATA not defined — provide no-op stubs */

void ble_raw_data_init(void)              {}
void ble_raw_data_set_enabled(bool e)     { (void)e; }

#endif /* ENABLE_BLE_RAW_DATA */
