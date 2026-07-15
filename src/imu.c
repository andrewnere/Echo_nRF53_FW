/*
 * imu.c — LSM6DSO direct-register driver with FIFO + hardware timestamps
 *
 * Bypasses the Zephyr sensor driver to access the LSM6DSO's on-chip FIFO
 * and 25 µs-resolution hardware timestamp counter directly over I2C.
 *
 * Why bypass the Zephyr sensor driver?
 *   The standard sensor API (sensor_sample_fetch / sensor_channel_get) reads
 *   the live output registers, which means:
 *     - Samples can be overwritten if the thread is delayed by BLE activity
 *     - Timestamps are software (kernel clock, ~1 ms resolution, jitter)
 *   Using the FIFO + hardware timestamps solves both: the sensor buffers data
 *   independently of the CPU, and each word carries its own precise timestamp.
 *
 *
 * FIFO configuration:
 *   - Continuous mode (oldest data discarded on overflow, never stalls)
 *   - Accel + gyro batched at the selected ODR (default set by IMU_RATE_IDX)
 *   - Hardware timestamp inserted every sample (decimation = 1)
 *   - FIFO depth: 512 words; 3 words per sample ~170 samples max
 *
 */

#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "imu.h"
#include "app_config.h"

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

/* ============================================================
 * I2C bus and device address
 * ============================================================ */

#define I2C_BUS_NODE DT_NODELABEL(arduino_i2c)
#define LSM6DSO_ADDR 0x6B /* SA0/SDO pin tied to VCC on this breakout */

static const struct device* i2c_bus;

/* ============================================================
 * LSM6DSO register addresses
 * ============================================================ */

#define REG_FIFO_CTRL1        0x07 /* FIFO watermark bits [7:0]                   */
#define REG_FIFO_CTRL2        0x08 /* FIFO watermark bit [8]; compression flags   */
#define REG_FIFO_CTRL3        0x09 /* Batch data rate: gyro[7:4], accel[3:0]      */
#define REG_FIFO_CTRL4        0x0A /* FIFO mode[2:0]; timestamp decimation[5:4]   */
#define REG_WHO_AM_I          0x0F /* Fixed ID register — always reads 0x6C       */
#define REG_CTRL1_XL          0x10 /* Accel: ODR[7:4], full-scale[3:2]            */
#define REG_CTRL2_G           0x11 /* Gyro:  ODR[7:4], full-scale[3:1], FS125[0] */
#define REG_CTRL3_C           0x12 /* BDU[6], IF_INC[2], SW_RESET[0]             */
#define REG_CTRL10_C          0x19 /* TIMESTAMP_EN[5]                             */
#define REG_FIFO_STATUS1      0x3A /* DIFF_FIFO[7:0] — unread word count LSB      */

#define REG_FIFO_DATA_OUT_TAG 0x78 /* Tag byte: sensor_id[7:3], cnt[2:1], par[0] */
/* 0x79–0x7E: X_L, X_H, Y_L, Y_H, Z_L, Z_H — read 7 bytes from 0x78 per word    */

/* ============================================================
 * FIFO tag values (upper 5 bits of the tag byte)
 * ============================================================ */

#define FIFO_TAG_GYRO      0x01 /* Gyroscope NC (non-compressed) sample  */
#define FIFO_TAG_ACCEL     0x02 /* Accelerometer NC sample               */
#define FIFO_TAG_TIMESTAMP 0x04 /* Hardware timestamp word               */

/* ============================================================
 * Register field values
 * ============================================================ */

/*
 * ODR codes — same encoding for CTRL1_XL[7:4], CTRL2_G[7:4],
 * and the batch-data-rate fields in FIFO_CTRL3.
 */
#define ODR_12_5HZ 0x1
#define ODR_26HZ   0x2
#define ODR_52HZ   0x3
#define ODR_104HZ  0x4
#define ODR_208HZ  0x5

/*
 * Accelerometer full-scale — CTRL1_XL bits [3:2].
 * Using ±2 g gives the best resolution (0.061 mg/LSB).
 */
#define ACCEL_FS_2G 0x00 /* bits[3:2] = 00 */

/*
 * Gyroscope full-scale — CTRL2_G bits [3:1] = FS_G, bit [1] = FS_125.
 *
 * Available options (written into CTRL2_G alongside the ODR field):
 *   GYRO_FS_125DPS  0x02   FS_125=1 → ±125  dps,  4.375 mdps/LSB  (0.004375 dps/LSB)
 *   GYRO_FS_500DPS  0x04   FS_G=01  → ±500  dps,  17.5  mdps/LSB  (0.0175   dps/LSB)
 */
#define GYRO_FS_500DPS 0x04 
#define GYRO_FS_ACTIVE GYRO_FS_500DPS
#define FIFO_CFG_CONTINUOUS_WITH_TS 0x46

/* WHO_AM_I value expected from LSM6DSO */
#define LSM6DSO_WHO_AM_I_VAL 0x6C

/* ============================================================
 * Rate configuration table
 * ============================================================ */

struct rate_cfg {
    uint8_t     odr;       /* ODR code for CTRL1_XL / CTRL2_G            */
    uint8_t     bdr;       /* Batch data rate code for FIFO_CTRL3        */
    uint32_t    period_us; /* Exact sample period in microseconds         */
    const char* label;     /* String for STATUS message and log           */
};

/*
 * Index 0 = RATE:1 (12.5 Hz) … index 4 = RATE:5 (208 Hz).
 * ODR and BDR codes are identical for the rates we use.
 * period_us = 1 000 000 / ODR_Hz (rounded to nearest µs).
 */
static const struct rate_cfg k_rates[] = {
    {ODR_12_5HZ, ODR_12_5HZ, 80000, "12.5"}, /* RATE:1 */
    {ODR_26HZ, ODR_26HZ, 38461, "26"},       /* RATE:2 */
    {ODR_52HZ, ODR_52HZ, 19230, "52"},       /* RATE:3 */
    {ODR_104HZ, ODR_104HZ, 9615, "104"},     /* RATE:4 */
    {ODR_208HZ, ODR_208HZ, 4808, "208"},     /* RATE:5 */
};

/* Shared between imu_thread and cmd_thread; written only from cmd_thread */
static volatile int  m_rate_idx       = IMU_RATE_IDX;
static volatile bool s_uart_raw_enabled = false;
static void        (*s_mode_cb)(int mode) = NULL;

void imu_set_uart_raw(bool enabled)          { s_uart_raw_enabled = enabled; }
void imu_set_mode_change_cb(void (*cb)(int)) { s_mode_cb = cb; }

/*
 * Free-running software timestamp counter (units: 25 µs ticks).
 * Tracks in the same unit as the hardware counter so the output path
 * is a direct pass-through with no arithmetic in either case.
 * Seeded once from kernel uptime; advanced by s_period_ticks per sample.
 */
static uint32_t s_ts_ticks;           /* next tick value to emit          */
static bool     s_ts_seeded;          /* true once the counter is seeded  */
static uint32_t s_period_ticks = 769; /* 52 Hz default: 19230 µs / 25    */

#define IMU_CB_SLOTS 4
static imu_sample_cb_t s_sample_cbs[IMU_CB_SLOTS];

int imu_add_sample_cb(imu_sample_cb_t cb) {
    for (int i = 0; i < IMU_CB_SLOTS; i++) {
        if (s_sample_cbs[i] == NULL) {
            s_sample_cbs[i] = cb;
            return 0;
        }
    }
    LOG_ERR("imu_add_sample_cb: all %d slots full", IMU_CB_SLOTS);
    return -ENOMEM;
}

/* ============================================================
 * I2C helper functions
 * ============================================================ */

/* Write a single byte to a register */
static int reg_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write(i2c_bus, buf, sizeof(buf), LSM6DSO_ADDR);
}

/* Read one byte from a register */
static int reg_read(uint8_t reg, uint8_t* val) {
    return i2c_write_read(i2c_bus, LSM6DSO_ADDR, &reg, 1, val, 1);
}

/* Read n consecutive bytes starting at reg (auto-increment) */
static int reg_read_burst(uint8_t reg, uint8_t* buf, size_t n) {
    return i2c_write_read(i2c_bus, LSM6DSO_ADDR, &reg, 1, buf, n);
}

/* ============================================================
 * LSM6DSO initialisation
 * ============================================================ */

/* Verify the sensor is present and responding */
static int lsm6dso_check_id(void) {
    uint8_t id;
    int     ret = reg_read(REG_WHO_AM_I, &id);

    if (ret != 0) {
        LOG_ERR("I2C read failed (ret=%d) — check wiring", ret);
        return ret;
    }
    if (id != LSM6DSO_WHO_AM_I_VAL) {
        LOG_ERR("WHO_AM_I=0x%02X, expected 0x%02X — wrong device or address",
                id, LSM6DSO_WHO_AM_I_VAL);
        return -ENODEV;
    }
    LOG_INF("LSM6DSO found (WHO_AM_I=0x%02X)", id);
    return 0;
}

/*
 * Apply the rate at index `idx` from k_rates[].
 */
static int lsm6dso_set_rate(int idx) {
    const struct rate_cfg* r = &k_rates[idx];
    int                    ret;

    /* Step 1: put FIFO in bypass to clear it */
    ret = reg_write(REG_FIFO_CTRL4, 0x00);
    if (ret)
        return ret;

    /* Step 2: set accel ODR — high-performance mode is default (CTRL6_C[4]=0) */
    ret = reg_write(REG_CTRL1_XL, (r->odr << 4) | ACCEL_FS_2G);
    if (ret)
        return ret;

    /* Step 3: set gyro ODR — high-performance mode is default (CTRL7_G[7]=0) */
    ret = reg_write(REG_CTRL2_G, (r->odr << 4) | GYRO_FS_ACTIVE);
    if (ret)
        return ret;

    /* Step 4: update batch data rates to match new ODR */
    ret = reg_write(REG_FIFO_CTRL3, (uint8_t)((r->bdr << 4) | r->bdr));
    if (ret)
        return ret;

    /* Step 5: re-enable continuous FIFO with timestamp every sample */
    ret = reg_write(REG_FIFO_CTRL4, FIFO_CFG_CONTINUOUS_WITH_TS);
    if (ret)
        return ret;

    /* Reset the software counter so it re-seeds at the new rate */
    s_period_ticks = r->period_us / 25U;
    s_ts_seeded = false;

#ifdef ENABLE_UART_DEBUGGING
    printk("STATUS:LSM6DSO,%s\n", r->label);
#endif
    return 0;
}

/* Full sensor + FIFO initialisation */
static int lsm6dso_init(void) {
    int ret;

    /* Software reset — clears all registers to default */
    ret = reg_write(REG_CTRL3_C, 0x01);
    if (ret)
        return ret;
    k_msleep(10); /* datasheet: wait ≥ 50 µs; 10 ms is comfortably safe */

    /*
     * CTRL3_C: BDU=1 (block data update — output registers not updated
     * until both bytes read), IF_INC=1 (auto-increment on burst reads).
     * Reset value already has IF_INC=1 (0x04); we OR in BDU (0x40).
     */
    ret = reg_write(REG_CTRL3_C, 0x44);
    if (ret)
        return ret;

    /* CTRL10_C: TIMESTAMP_EN=1 (bit 5) — start the hardware timestamp counter */
    ret = reg_write(REG_CTRL10_C, 0x20);
    if (ret)
        return ret;


    ret = reg_write(REG_FIFO_CTRL1, 30); /* WTM[7:0] */
    if (ret)
        return ret;
    ret = reg_write(REG_FIFO_CTRL2, 0x00); /* WTM[8]=0, no compression */
    if (ret)
        return ret;

    /* Apply the configured rate (IMU_RATE_IDX), which also enables the FIFO */
    ret = lsm6dso_set_rate(m_rate_idx);
    if (ret)
        return ret;

#ifdef ENABLE_UART_DEBUGGING
    printk("# timestamp,ax,ay,az,gx,gy,gz\n");
    printk("# timestamp: 25 µs/tick  → ts_us=tick*25, ts_ms=tick*0.025\n");
    printk("# ax/ay/az:  raw int16,  scale 0.061 mg/LSB      (±2 g)\n");
    printk("# gx/gy/gz:  raw int16,  scale 0.0175 dps/LSB (±500 dps)\n");
#endif
    return 0;
}

/* ============================================================
 * FIFO drain
 * ============================================================ */

/*
 * Accumulator for one complete {accel, gyro, timestamp} set.
 * The LSM6DSO inserts the three word types in a consistent order but
 * we track them with flags so the code is robust to any ordering.
 */
struct fifo_sample {
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    uint32_t ts_ticks; /* raw 25 µs ticks straight from sensor FIFO word */
    bool     has_accel;
    bool     has_gyro;
    bool     has_ts;
};

/*
 * Output one complete sample and reset the accumulator.
 *
 * Timestamp source priority:
 *   1. Hardware (has_ts=true): raw 25 µs ticks from the sensor FIFO word,
 *      passed straight to the output.  Also syncs the software counter so
 *      the fallback stays accurate if HW timestamps ever drop out.
 *   2. Software fallback (has_ts=false): free-running counter seeded once
 *      from kernel uptime, advanced by exactly s_period_us per sample.
 *      Divided by 25 at output to match the hardware tick format.
 *
 * Output format: tick,ax,ay,az,gx,gy,gz
 *   tick × 25  = µs,  tick × 0.025 = ms,  tick × 0.000025 = s
 */
static void maybe_output(struct fifo_sample* s) {
    if (!s->has_accel || !s->has_gyro) {
        return;
    }

    uint32_t ts_ticks;

    if (s->has_ts) {
        /* Hardware tick counter — straight from the sensor, no math */
        ts_ticks = s->ts_ticks;
        /* Keep software counter in sync in case HW timestamps drop out */
        s_ts_ticks = ts_ticks + s_period_ticks;
        s_ts_seeded = true;
    } else {
        /* Software fallback — also tracked in ticks, no conversion at output */
        if (!s_ts_seeded) {
            /* 1 ms = 40 ticks (1000 µs / 25 µs per tick) */
            s_ts_ticks = k_uptime_get_32() * 40U;
            s_ts_seeded = true;
        }
        ts_ticks = s_ts_ticks;
        s_ts_ticks += s_period_ticks;
    }

    if (s_uart_raw_enabled) {
        printk("%u,%d,%d,%d,%d,%d,%d\n",
               ts_ticks,
               s->ax, s->ay, s->az,
               s->gx, s->gy, s->gz);
    }

    struct imu_sample out = {
        .ts_ticks = ts_ticks,
        .ax = s->ax,
        .ay = s->ay,
        .az = s->az,
        .gx = s->gx,
        .gy = s->gy,
        .gz = s->gz,
    };
    for (int i = 0; i < IMU_CB_SLOTS; i++) {
        if (s_sample_cbs[i] != NULL) {
            s_sample_cbs[i](&out);
        }
    }

    s->has_accel = s->has_gyro = s->has_ts = false;
}

/*
 * Read however many words are currently in the FIFO and emit complete
 * samples as CSV.  Called from the IMU thread on every wake.
 */
static void fifo_drain(void) {
    uint8_t status[2];
    int     ret;

    /* Read FIFO fill count (9-bit value across two registers) */
    ret = reg_read_burst(REG_FIFO_STATUS1, status, 2);
    if (ret != 0) {
        LOG_ERR("FIFO_STATUS read failed: %d", ret);
        return;
    }

    uint16_t count = (uint16_t)status[0] | (((uint16_t)(status[1] & 0x01)) << 8);

#ifdef ENABLE_UART_DEBUGGING
    if (status[1] & 0x40) {
        /* FIFO has wrapped — data was lost.  Rare at 10 ms poll + 208 Hz. */
        printk("WARN:FIFO_OVERRUN\n");
    }
#endif

    if (count == 0) {
        return;
    }

    static struct fifo_sample pending; /* carries state across calls */

    for (uint16_t i = 0; i < count; i++) {
        /*
         * Each FIFO word is 7 bytes:
         *   byte 0   : tag byte  (sensor_id[7:3] | counter[2:1] | parity[0])
         *   bytes 1-6: X_L, X_H, Y_L, Y_H, Z_L, Z_H
         *
         * Reading 7 bytes from REG_FIFO_DATA_OUT_TAG (0x78) with
         * auto-increment pops one word from the FIFO.
         */
        uint8_t word[7];
        ret = reg_read_burst(REG_FIFO_DATA_OUT_TAG, word, sizeof(word));
        if (ret != 0) {
            LOG_ERR("FIFO read failed at word %u: %d", i, ret);
            break;
        }

        uint8_t tag = word[0] >> 3; /* upper 5 bits = sensor identifier */

        switch (tag) {

        case FIFO_TAG_ACCEL:
            /* Reconstruct signed 16-bit values from little-endian bytes */
            pending.ax = (int16_t)((uint16_t)word[1] | ((uint16_t)word[2] << 8));
            pending.ay = (int16_t)((uint16_t)word[3] | ((uint16_t)word[4] << 8));
            pending.az = (int16_t)((uint16_t)word[5] | ((uint16_t)word[6] << 8));
            pending.has_accel = true;
            break;

        case FIFO_TAG_GYRO:
            pending.gx = (int16_t)((uint16_t)word[1] | ((uint16_t)word[2] << 8));
            pending.gy = (int16_t)((uint16_t)word[3] | ((uint16_t)word[4] << 8));
            pending.gz = (int16_t)((uint16_t)word[5] | ((uint16_t)word[6] << 8));
            pending.has_gyro = true;
            break;

        case FIFO_TAG_TIMESTAMP:
            /*
             * 32-bit tick counter packed little-endian into bytes 1-4.
             * Bytes 5-6 are always 0x00.  Each tick = 25 µs.
             * Store raw ticks — no conversion needed, we output them directly.
             */
            pending.ts_ticks =
                (uint32_t)word[1] |
                ((uint32_t)word[2] << 8) |
                ((uint32_t)word[3] << 16) |
                ((uint32_t)word[4] << 24);
            pending.has_ts = true;
            break;

        default:
            /* Temperature or other tag — not used, skip */
            break;
        }

        maybe_output(&pending);
    }
}

/* ============================================================
 * IMU thread — drains FIFO and prints CSV
 * ============================================================ */

static void imu_thread_fn(void* a, void* b, void* c) {
    i2c_bus = DEVICE_DT_GET(I2C_BUS_NODE);

    if (!device_is_ready(i2c_bus)) {
        LOG_ERR("I2C bus not ready");
        return;
    }

    if (lsm6dso_check_id() != 0) {
        return;
    }

    if (lsm6dso_init() != 0) {
        LOG_ERR("LSM6DSO init failed");
        return;
    }

    LOG_INF("IMU streaming started");

    while (1) {
        /* Poll every 5 ms — well under any supported ODR period,
         * prevents FIFO overrun even if BLE briefly stalls this thread. */
        k_msleep(5);
        fifo_drain();
    }
}

K_THREAD_DEFINE(imu_tid, 4096, imu_thread_fn, NULL, NULL, NULL, 7, 0, 0);

/* ============================================================
 * Command thread — parses RATE:N from UART
 * ============================================================ */

static void cmd_thread_fn(void* a, void* b, void* c) {
    const struct device* uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    char                 buf[16];
    int                  idx = 0;
    unsigned char        ch;

    while (1) {
        if (uart_poll_in(uart, &ch) != 0) {
            k_msleep(10);
            continue;
        }

        if (ch == '\n' || ch == '\r') {
            if (idx == 0) {
                continue;
            }

            buf[idx] = '\0';
            idx = 0;

            /*
             * RATE:N  where N is '1'–'5'
             * Maps to index 0–4 in k_rates[].
             */
            if (strncmp(buf, "RATE:", 5) == 0 && buf[5] != '\0' && buf[6] == '\0') {
                int level = buf[5] - '1'; /* '1'→0, '5'→4 */

                if (level >= 0 && level < (int)ARRAY_SIZE(k_rates)) {
                    m_rate_idx = level;
                    lsm6dso_set_rate(level);
                } else {
                    printk("ERROR:bad rate, use RATE:1 to RATE:5\n");
                }

            } else if (strncmp(buf, "MODE:", 5) == 0 && buf[5] != '\0' && buf[6] == '\0') {
                int mode = buf[5] - '0'; /* '0'→0 … '3'→3 */

                if (mode >= 0 && mode <= 3) {
                    if (s_mode_cb) {
                        s_mode_cb(mode);
                    }
                    printk("STATUS:MODE:%d\n", mode);
                } else {
                    printk("ERROR:bad mode — use MODE:0 (UART), MODE:1 (HID), MODE:2 (Raw), MODE:3 (Both)\n");
                }

            } else {
                printk("ERROR:unknown command\n");
            }
        } else if (idx < (int)sizeof(buf) - 1) {
            buf[idx++] = ch;
        }
    }
}

K_THREAD_DEFINE(cmd_tid, 1024, cmd_thread_fn, NULL, NULL, NULL, 7, 0, 0);
