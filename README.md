# Drone Tracking and Telemetry IoT System

Embedded telemetry and status-monitoring node for a drone platform, built on the **STM32WL55JC (`NUCLEO-WL55JC1`)** microcontroller board running **Zephyr RTOS** (*Embedded Platforms and Communications for IoT*, Universidad Politécnica de Madrid).

The node acquires 6-axis inertial data, GPS position and UTC time, ambient light, and environmental temperature/humidity, classifies the operating state (`IDLE`, `FLYING`, `PROBLEM`), drives an RGB status indicator, and transmits formatted telemetry over the ST-LINK Virtual COM port (`115200 8N1`), including the deterministic `10 Hz` `SIMU` frame interface for the flight simulator.

## Hardware Connections & Peripheral Configuration

| Subsystem | Device / Signal | MCU Pin (Morpho) | Peripheral & Configuration |
| :--- | :--- | :--- | :--- |
| **IMU** | MPU6050 (`0x68`) | `PB8` SCL (CN10-27)<br>`PB9` SDA (CN10-26) | `I2C1` @ `100 kHz`; `PWR_MGMT_1=0x09` (X-gyro PLL, temp off), `CONFIG=0x05` (`10 Hz` DLPF), `±2 g`, `±250 °/s`, sampled at `25 Hz` |
| **Temp / RH** | Si7021 (`0x40`) | `PB8` SCL, `PB9` SDA | Shared `I2C1` (mutex-protected); No-Hold Master RH (`0xF5`) + Read Previous Temp (`0xE0`), sampled every `2000 ms` |
| **GPS** | PA6H Breakout | `PB6` TX (CN10-35)<br>`PB7` RX (CN10-37) | `USART1` @ `9600 8N1` (interrupt-driven RX); configured via `$PMTK314` for GGA-only output (`$GPGGA` / `$GNGGA`), processed every `1000 ms` |
| **Ambient Light** | LDR Voltage Divider | `PB1` / `A0` (CN7-28) | `ADC1_IN5`, `12-bit` (`0–4095`), internal reference, max acquisition time (`160.5` cycles), mapped to `0–100%`, sampled every `2000 ms` |
| **Status LED** | Common-Cathode RGB | `PB11` R (CN10-30)<br>`PB10` G (CN10-25)<br>`PB15` B (CN10-28) | Active-high GPIO digital outputs |
| **Mode Button** | On-board `B1` (`sw0`) | `PA0` | GPIO edge interrupt with `200 ms` software debounce |
| **Telemetry UART** | ST-LINK VCOM | On-board `LPUART1` | `115200 8N1`, interrupt-driven non-blocking TX |

## Operating Modes & LED Indicators

Pressing button **`B1` (`PA0`)** cycles through the operating modes:

| Mode | Output Period | RGB LED Color | Description |
| :--- | :---: | :--- | :--- |
| **TEST** *(boot default)* | `2000 ms` | **Blue** | Sensor and peripheral diagnostic report with current measurements and status flags. |
| **NORMAL** | `5000 ms` | **Green** (`IDLE`)<br>**Yellow** (`FLYING`)<br>**Red** (`PROBLEM`) | Operational telemetry summary including 30-second rolling window statistics (`mean`, `min`, `max`). |
| **SIMU** | `100 ms` (`10 Hz`) | **Magenta** | Fixed-width `221-byte` ASCII frame (`$SIMU,...*CS\r\n`) with XOR checksum and hexadecimal `FLAGS` bitmask for the Python flight simulator. |

## Zephyr RTOS Architecture

The firmware avoids busy-wait polling so the CPU enters idle sleep (`WFI`) between sensor and timer events:

- **`processing_thread` (Priority 4, 2048 B)**: Consumes `imu_event_queue`, performs a `2000 ms` start-up still calibration, runs complementary roll/pitch estimation, evaluates the hysteresis/persistence state machine (`IDLE` / `FLYING` / `PROBLEM`), updates local position/velocity estimates, and maintains 30-second rolling statistics.
- **`imu_thread` (Priority 5, 1536 B)**: Samples the MPU6050 every `40 ms` (`25 Hz`) and pushes events to `imu_event_queue`.
- **`gps_thread` (Priority 6, 1536 B)**: Parses the latest interrupt-captured GGA sentence every `1000 ms` with a `3000 ms` staleness timeout.
- **`environment_thread` (Priority 7, 1536 B)**: Reads the Si7021 and LDR ADC channel every `2000 ms`.
- **`output_thread` (Priority 8, 2048 B)**: Triggered by `output_timer` at the active mode period; formats and transmits reports/frames over interrupt-driven VCOM UART.
- **`mode_thread` (Priority 9, 1024 B)**: Blocks on `mode_button_queue`, applies button debouncing, updates the active mode and LED state, and reconfigures `output_timer`.

Shared telemetry state is protected by `snapshot_mutex`, shared `I2C1` transactions by `i2c1_bus_mutex`, and LED updates by `led_mutex`.

## Repository Structure

```text
├── app.overlay          # NUCLEO-WL55JC1 DeviceTree pinout and peripheral overlay
├── prj.conf             # Zephyr kernel and driver configuration
├── Kconfig              # Application sampling rates, LDR calibration, and state thresholds
├── CMakeLists.txt       # Zephyr build definition
├── include/             # Module headers and shared telemetry snapshot/flag contracts
├── src/                 # Register-level sensor drivers, processing pipeline, mode & output logic
├── docs/                # Architecture, interface, and SIMU protocol technical notes
└── tools/               # Host-side SIMU frame validator (check_simu.py) and unit checks
```

## Build, Flash & Verification

### 1. Build and Flash
From an initialized Zephyr workspace (`west` environment active):

```bash
west build -p always -b nucleo_wl55jc /path/to/Drone-Tracking-and-Telemetry-IoT-System
west flash
```

Connect a serial terminal (or the Python flight simulator in `SIMU` mode) to the **ST-LINK Virtual COM Port** at **`115200` baud, `8N1`**. Keep the board stationary and level for the first `2 s` after reset so start-up IMU calibration completes (`FLAGS` bit 5 clears).

### 2. Key Configuration Options (`Kconfig`)
- `CONFIG_APP_LDR_DARK_RAW` / `CONFIG_APP_LDR_MAX_RAW`: Raw 12-bit ADC endpoints for `0%` and `100%` ambient light calibration (defaults: `150` and `3600`).
- `CONFIG_APP_USE_MOCK_SENSORS`: Set to `y` only for host/board testing without external sensors attached (default `n`).
- `CONFIG_ASSERT`: Enabled in `prj.conf` during development; set `CONFIG_ASSERT=n` if assertion output on VCOM must be suppressed during simulator runs.

### 3. Host-Side SIMU Stream Validation
To validate a captured `SIMU` binary stream (`221` bytes/frame, `CRLF`, field widths, XOR checksum, and `100 ms` cadence):

```bash
python tools/check_simu.py capture.bin
```

## Authors

- Tu Xianlong
- Dhanashri Jasutkar
