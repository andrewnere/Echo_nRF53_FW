# nRF Virtual Mouse

BLE HID mouse driven by an LSM6DSO IMU on the nRF5340-DK.  
Sensor fusion (Madgwick/Game-Rotation-Vector style AHRS) converts head/hand tilt into cursor movement.  
Optionally streams raw IMU data over a second BLE connection for logging/analysis.  
Optionally also acts as a BLE **Central**, connecting out to an ESP32 peripheral and mapping incoming commands to onboard LEDs.

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

The BLE Central link to an ESP32 (see below) is independent of these modes — when enabled it scans/connects continuously regardless of the current mode.

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
#define ENABLE_BLE_HID           // HID mouse profile — "Virtual Mouse"
#define ENABLE_BLE_RAW_DATA      // Raw IMU GATT service — "VM-Raw"
#define ENABLE_NRF53_AS_CENTRAL  // BLE Central link out to an ESP32 (see below)
// Comment out any of these to strip that feature from the build entirely.
// If all three are commented out, BLE is not initialised at all.
```

### Board mounting orientation
```c
#define IMU_MOUNT_Y_DOWN   // or IMU_MOUNT_Z_UP — define exactly one
```
Required — there is no runtime auto-detection. Picking neither is a compile error.

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

### Debugging
```c
//#define ENABLE_UART_DEBUGGING  // uncomment for verbose [cal]/[gyro]/[imu] output
```

---

## BLE Devices

Two peripheral identities advertise simultaneously (extended advertising, one adv set each), plus one optional outgoing central link:

| Name | Identity | Role | Profile | Who connects |
|---|---|---|---|---|
| `Virtual Mouse` | 0 | Peripheral | HID over GATT | macOS / Windows — appears as a standard mouse |
| `VM-Raw` | 1 | Peripheral | Custom GATT notify | Python client (`ble_imu_client.py`) |
| *(ESP32's own name)* | — | Central | Custom GATT client | nRF53 connects out to an ESP32 running the paired sketch |

> **Note:** `VM-Raw` does not appear in macOS Bluetooth Settings — macOS only surfaces devices with recognised profiles (HID, audio). Use nRF Connect or the Python client to connect.

Each BLE role (HID, Raw, Central) is fully self-contained in its own source file — `src/ble_hid.c`, `src/ble_raw_data.c`, `src/ble_central.c` — with its own advertising/scanning and connection lifecycle. `src/main.c` only composes them and dispatches mode changes.

---

## BLE Central — ESP32 Command Link (`ENABLE_NRF53_AS_CENTRAL`)

When enabled, the nRF53 additionally acts as a BLE **Central**: it scans for, connects to, and subscribes to notifications from an ESP32 GATT server, then maps each incoming ASCII command to one of the DK's 4 LEDs (LED1-LED4 / `led0`-`led3`).

**ESP32-side service (must match):**
```c
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
const char* commands[] = {"cmd1", "cmd2", "cmd3", "cmd4"};
```

| Command | LED | Behavior |
|---|---|---|
| `cmd1` | LED1 (`led0`) | Toggle |
| `cmd2` | LED2 (`led1`) | Toggle |
| `cmd3` | LED3 (`led2`) | Toggle |
| `cmd4` | LED4 (`led3`) | Toggle |

Each command **toggles** its LED's current state (not exclusive — multiple LEDs can be on at once). Scanning restarts automatically on disconnect. This link runs independently of the IMU output modes above and of the two peripheral identities — see `src/ble_central.c`.

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
Key settings for dual advertising + the optional central link:

```
CONFIG_BT_CTLR_ADV_SET=2
CONFIG_BT_CTLR_ADV_EXT=y
CONFIG_BT_MAX_CONN=4
CONFIG_BT_CTLR_SDC_PERIPHERAL_COUNT=2
```

`CONFIG_BT_MAX_CONN` must match the app core's `prj.conf` value. It covers: the HID connection, the Raw connection, the ESP32 central connection, and one pending-adv pre-allocation slot (Zephyr reserves a `bt_conn` when starting connectable advertising).

Do not reduce `PERIPHERAL_COUNT` below 2 — each connectable advertising set consumes one peripheral slot, and dropping to 1 causes the second `bt_le_ext_adv_start` to fail with `-ENOMEM`. The SDC's central-link count is derived as `BT_MAX_CONN - BT_CTLR_SDC_PERIPHERAL_COUNT`, so it must stay ≥ 1 whenever `ENABLE_NRF53_AS_CENTRAL` is used.
