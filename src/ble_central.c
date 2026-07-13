/*
 * ble_central.c — nRF53 as BLE Central, connected to an ESP32 peripheral.
 *
 * Scans for an ESP32 GATT server advertising SERVICE_UUID, connects,
 * discovers CHARACTERISTIC_UUID, subscribes to its notifications, and maps
 * each incoming ASCII command ("cmd1".."cmd4") to one of the DK's 4 LEDs.
 *
 * This link is fully independent of the HID / raw-IMU peripheral roles in
 * main.c — separate scan/connect/GATT-client state, own connection
 * callbacks (filtered to BT_CONN_ROLE_CENTRAL so the peripheral links are
 * left untouched).
 */

#include "app_config.h"

#ifdef ENABLE_NRF53_AS_CENTRAL

#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "ble_central.h"

LOG_MODULE_REGISTER(ble_central, LOG_LEVEL_INF);

/* ============================================================
 * ESP32 peer — service / characteristic UUIDs and command set
 * ============================================================ */

#define BT_UUID_ESP32_SVC_VAL \
    BT_UUID_128_ENCODE(0x4fafc201, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914bULL)
#define BT_UUID_ESP32_CHRC_VAL \
    BT_UUID_128_ENCODE(0xbeb5483e, 0x36e1, 0x4688, 0xb7f5, 0xea07361b26a8ULL)

static struct bt_uuid_128 esp32_svc_uuid  = BT_UUID_INIT_128(BT_UUID_ESP32_SVC_VAL);
static struct bt_uuid_128 esp32_chrc_uuid = BT_UUID_INIT_128(BT_UUID_ESP32_CHRC_VAL);

static const char * const commands[] = { "cmd1", "cmd2", "cmd3", "cmd4" };

/* Silkscreen LED1-LED4 == devicetree aliases led0-led3. */
static const struct gpio_dt_spec cmd_leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios),
};

BUILD_ASSERT(ARRAY_SIZE(cmd_leds) == ARRAY_SIZE(commands),
             "one LED per command");

static void handle_command(const uint8_t *data, uint16_t len)
{
    for (size_t i = 0; i < ARRAY_SIZE(commands); i++) {
        size_t cmd_len = strlen(commands[i]);
        if (len == cmd_len && memcmp(data, commands[i], cmd_len) == 0) {
            int state = gpio_pin_toggle_dt(&cmd_leds[i]);
            printk("CENTRAL_CMD: %s -> LED%u %s\n", commands[i], (unsigned)(i + 1),
                   state == 0 ? "toggled" : "toggle failed");
            return;
        }
    }
    printk("CENTRAL_CMD: unrecognized (%u bytes)\n", len);
}

/* ============================================================
 * GATT discovery + subscribe
 * ============================================================ */

static struct bt_gatt_discover_params  discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

static uint8_t notify_func(struct bt_conn *conn,
                            struct bt_gatt_subscribe_params *params,
                            const void *data, uint16_t length)
{
    ARG_UNUSED(conn);

    if (!data) {
        printk("CENTRAL: unsubscribed\n");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    handle_command((const uint8_t *)data, length);
    return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              struct bt_gatt_discover_params *params)
{
    int err;

    if (!attr) {
        printk("CENTRAL: discovery failed/complete\n");
        (void)memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    if (!bt_uuid_cmp(discover_params.uuid, &esp32_svc_uuid.uuid)) {
        discover_params.uuid         = &esp32_chrc_uuid.uuid;
        discover_params.start_handle = attr->handle + 1;
        discover_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

        err = bt_gatt_discover(conn, &discover_params);
        if (err) { printk("CENTRAL: characteristic discover failed (%d)\n", err); }
    } else if (!bt_uuid_cmp(discover_params.uuid, &esp32_chrc_uuid.uuid)) {
        subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

        discover_params.uuid         = BT_UUID_GATT_CCC;
        discover_params.start_handle = attr->handle + 2;
        discover_params.type         = BT_GATT_DISCOVER_DESCRIPTOR;

        err = bt_gatt_discover(conn, &discover_params);
        if (err) { printk("CENTRAL: CCC discover failed (%d)\n", err); }
    } else {
        subscribe_params.notify     = notify_func;
        subscribe_params.value      = BT_GATT_CCC_NOTIFY;
        subscribe_params.ccc_handle = attr->handle;

        err = bt_gatt_subscribe(conn, &subscribe_params);
        if (err && err != -EALREADY) {
            printk("CENTRAL: subscribe failed (%d)\n", err);
        } else {
            printk("CENTRAL: subscribed\n");
        }
        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_STOP;
}

static void start_discovery(struct bt_conn *conn)
{
    discover_params.uuid         = &esp32_svc_uuid.uuid;
    discover_params.func         = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type         = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) { printk("CENTRAL: service discover failed (%d)\n", err); }
}

/* ============================================================
 * Scan + connect
 * ============================================================ */

static struct bt_conn *esp32_conn;

static void start_scan(void);

static bool uuid_match_cb(struct bt_data *data, void *user_data)
{
    bool *found = user_data;

    if (data->type != BT_DATA_UUID128_SOME && data->type != BT_DATA_UUID128_ALL) {
        return true; /* keep parsing */
    }

    for (uint16_t i = 0; i + 16 <= data->data_len; i += 16) {
        if (memcmp(&data->data[i], esp32_svc_uuid.val, 16) == 0) {
            *found = true;
            return false; /* stop parsing */
        }
    }
    return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                          struct net_buf_simple *ad)
{
    ARG_UNUSED(rssi);

    if (esp32_conn) {
        return;
    }
    if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        return;
    }

    bool found = false;
    bt_data_parse(ad, uuid_match_cb, &found);
    if (!found) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    printk("CENTRAL: ESP32 found (%s), connecting\n", addr_str);

    if (bt_le_scan_stop()) {
        return;
    }

    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                                 BT_LE_CONN_PARAM_DEFAULT, &esp32_conn);
    if (err) {
        printk("CENTRAL: create conn failed (%d)\n", err);
        start_scan();
    }
}

static void start_scan(void)
{
    struct bt_le_scan_param scan_param = {
        .type     = BT_LE_SCAN_TYPE_ACTIVE,
        .options  = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window   = BT_GAP_SCAN_FAST_WINDOW,
    };

    int err = bt_le_scan_start(&scan_param, device_found);
    if (err == 0)              { printk("CENTRAL: scanning for ESP32\n"); }
    else if (err != -EALREADY) { printk("CENTRAL: scan start failed (%d)\n", err); }
}

/* ============================================================
 * Connection callbacks — filtered to this link only (BT_CONN_ROLE_CENTRAL);
 * the HID / raw-IMU peripheral links are handled by main.c's own callback.
 * ============================================================ */

static void central_connected(struct bt_conn *conn, uint8_t err)
{
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    if (info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }

    if (err) {
        printk("CENTRAL: connect failed (%u)\n", err);
        bt_conn_unref(esp32_conn);
        esp32_conn = NULL;
        start_scan();
        return;
    }

    printk("CENTRAL: connected to ESP32\n");
    start_discovery(conn);
}

static void central_disconnected(struct bt_conn *conn, uint8_t reason)
{
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    if (info.role != BT_CONN_ROLE_CENTRAL || conn != esp32_conn) {
        return;
    }

    printk("CENTRAL: disconnected from ESP32 (reason 0x%02x)\n", reason);
    bt_conn_unref(esp32_conn);
    esp32_conn = NULL;
    start_scan();
}

BT_CONN_CB_DEFINE(central_conn_callbacks) = {
    .connected    = central_connected,
    .disconnected = central_disconnected,
};

/* ============================================================
 * Init
 * ============================================================ */

void ble_central_init(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(cmd_leds); i++) {
        if (!gpio_is_ready_dt(&cmd_leds[i])) {
            printk("CENTRAL: LED%u not ready\n", (unsigned)(i + 1));
            continue;
        }
        gpio_pin_configure_dt(&cmd_leds[i], GPIO_OUTPUT_INACTIVE);
    }

    start_scan();
}

#endif /* ENABLE_NRF53_AS_CENTRAL */
