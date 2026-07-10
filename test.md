<<<<<<< HEAD
# Echo_nRF53_FW
=======
# nRF Virtual Mouse

BLE HID mouse driven by an LSM6DSO IMU on the nRF5340-DK.  
Sensor fusion (Madgwick AHRS) converts head/hand tilt into cursor movement.  
Optionally streams raw IMU data over a second BLE connection for logging/analysis.

---

## Hardware

| Component | Detail |
|---|---|
| MCU | Nordic nRF5340-DK |
| IMU | STMicro LSM6DSO (I²C, QWIIC/Arduino header) |
| Connection | SDA → D14 (P1.02), SCL → D15 (P1.03) |

---

## Firmware

**Toolchain:** nRF Connect SDK 2.6.1 / Zephyr RTOS  
**Build system:** west

```
west build -b nrf5340dk_nrf5340_cpuapp
west flash
```

UART console: **460800 baud** (RTT or USB-UART adapter).

---

## Output Modes

Four runtime modes control what the device does. Set the default at compile time or change it live over UART.

| Mode | Name | Description |
|---|---|---|
| `0` | UART | Raw IMU CSV on UART. No BLE. |
| `1` | HID | BLE HID mouse only. Pairs as a standard mouse on macOS/Windows. |
| `2` | Raw | BLE raw IMU stream only. Connect with the Python client. |
| `3` | Both | HID mouse + raw IMU stream simultaneously. |

---

## UART Commands

Send ASCII commands terminated with `\n` or `\r` at 460800 baud.

### Set output mode
```
MODE:0    # UART CSV (no BLE)
MODE:1    # BLE HID mouse only
MODE:2    # BLE raw IMU stream only
MODE:3    # BLE HID + raw IMU stream
```

### Set IMU sample rate
```
RATE:1    # 12.5 Hz
RATE:2    #   26 Hz
RATE:3    #   52 Hz  ← default
RATE:4    #  104 Hz
RATE:5    #  208 Hz
```

Rate changes take effect immediately — no reboot needed. The FIFO is flushed before the new rate is applied.

---

## Compile-time Configuration (`src/app_config.h`)

All tunable parameters live in one place.

### Feature guards
```c
#define ENABLE_BLE_HID       // HID mouse profile — "Virtual Mouse"
#define ENABLE_BLE_RAW_DATA  // Raw IMU GATT service — "VM-Raw"
// Comment out either to strip that feature from the build entirely.
// If both are commented out, BLE is not initialised at all.
```

### Boot mode
```c
#define DEFAULT_OUTPUT_MODE  3   // 0=UART, 1=HID, 2=Raw, 3=Both
```

### IMU sample rate
```c
#define IMU_RATE_IDX  2   // 0=12.5 Hz, 1=26 Hz, 2=52 Hz, 3=104 Hz, 4=208 Hz
// IMU_ODR_HZ is derived automatically — do not edit it.
```

### Mouse feel
```c
#define MOUSE_SENSITIVITY_X   50.0f   // pixels per degree of yaw
#define MOUSE_SENSITIVITY_Y   50.0f   // pixels per degree of pitch
#define MOUSE_DEADZONE_X      0.1f    // degrees — motion below this is ignored
#define MOUSE_DEADZONE_Y      0.1f
#define MOUSE_SMOOTH_TAU      0.06f   // EMA time constant (seconds)
```

### Calibration timing
```c
#define IMU_CAL_SETTLE_S   3   // seconds to discard at startup
#define IMU_CAL_COLLECT_S  3   // seconds to accumulate gyro bias + gravity
// Total calibration window: ~6 seconds. Hold the device still.
```

### BLE raw batch size
```c
#define BLE_RAW_BATCH_SIZE  3  // IMU samples per GATT notification (3 × 16 = 48 bytes)
```

### Debugging
```c
//#define ENABLE_UART_DEBUGGING  // uncomment for verbose [cal]/[gyro]/[imu] output
```

---

## BLE Devices

Two separate BLE identities advertise simultaneously (extended advertising, one adv set each).

| Name | Identity | Profile | Who connects |
|---|---|---|---|
| `Virtual Mouse` | 0 | HID over GATT | macOS / Windows — appears as a standard mouse |
| `VM-Raw` | 1 | Custom GATT notify | Python client (`ble_imu_client.py`) |

> **Note:** `VM-Raw` does not appear in macOS Bluetooth Settings — macOS only surfaces devices with recognised profiles (HID, audio). Use nRF Connect or the Python client to connect.

---

## Python Client (`ble_imu_client.py`)

Connects to `VM-Raw` and streams raw IMU data. Writes `imu_trace.csv` (overwritten each run).

**Dependencies:**
```
pip install bleak
```

**Run:**
```
python ble_imu_client.py
```

**CSV columns:** `time_ms, ax, ay, az, gx, gy, gz`  
Scale factors: accel × 0.061 = mg (±2 g), gyro × 0.0175 = dps (±500 dps).

Firmware must be in mode 2 or 3 (`MODE:2` or `MODE:3`) for raw data to stream.

---

## Raw IMU CSV Format (UART, Mode 0)

One line per sample:
```
timestamp,ax,ay,az,gx,gy,gz
```
`timestamp` is in 25 µs ticks: `tick × 0.025 = ms`.

---

## Network Core

`child_image/hci_ipc.conf` configures the SDC controller on the network core.  
Key settings for dual advertising:

```
CONFIG_BT_CTLR_ADV_SET=2
CONFIG_BT_CTLR_ADV_EXT=y
CONFIG_BT_MAX_CONN=3
CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT=2
```

Do not reduce `PERIPHERAL_COUNT` — each connectable advertising set consumes one peripheral slot, and dropping to 1 causes the second `bt_le_ext_adv_start` to fail with `-ENOMEM`.
>>>>>>> a84403b (first commit)
