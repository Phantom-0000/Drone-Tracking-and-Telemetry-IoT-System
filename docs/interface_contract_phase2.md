# Phase 2 Interface and Data Contract

## Purpose

This document freezes the application-facing contract between A's low-level sensor modules and B's Zephyr application. It defines the units, timestamps, validity rules, error model, fallback behavior, and shared telemetry model required before the application skeleton is implemented.

Raw register access and bus-specific details remain inside the sensor modules. The contract below exposes normalized measurements to the application so that processing, mode handling, and output do not depend on a particular driver implementation.

## Ownership boundary

| Area | A provides | B owns |
|---|---|---|
| MPU6050 | I2C transactions, register configuration, raw reads, isolated tests | Sampling schedule, timestamping, normalized sample publication, error aggregation |
| GPS PA6H | UART receive path, NMEA parsing support, isolated tests | Parser integration, latest-value snapshot, fix/UTC validity handling |
| Si7021 | I2C commands, raw conversion support, isolated tests | Environment acquisition schedule and publication |
| LDR | ADC access and raw reading, isolated tests | Dark/maximum calibration, percentage mapping, saturation handling |
| RGB LED and B1 | GPIO/PWM and input primitives | Mode/state policy and debounced transitions |

The application boundary uses physical units and explicit validity. A driver may retain raw values internally, but it must not force processing or output code to repeat register-level conversions.

## Common result and timestamp rules

- Every successful sample carries a monotonic `timestamp_ms` measured from system boot.
- The timestamp is the measurement completion time for polled sensors and the receive/parse time for GPS data.
- GPS UTC is a separate wall-clock value. It is valid only when the parsed sentence supplies a valid UTC time.
- `SENSOR_OK` with `gps_fix == false` means that GPS communication and parsing are working but there is no fix. This condition alone is not a system error and must not force `PROBLEM`.
- A failed read returns a non-zero `sensor_result` and sets the corresponding telemetry flag. The application remains runnable.
- The application retains the last valid value after a temporary failure. If no valid value exists, it publishes a formatted zero through the output layer.
- Sensor functions must return within their configured deadline; they must not wait indefinitely for a bus or sentence.

## Sensor-facing contracts

The following signatures describe the stable application-facing behavior. Exact driver names may differ, but the argument and result semantics must remain equivalent.

```c
int mpu6050_read_sample(struct imu_sample *out);
int gps_parser_feed(const uint8_t *data, size_t length);
int gps_get_latest(struct gps_sample *out);
int si7021_read_sample(struct environment_sample *out);
int ldr_read_raw(struct ldr_sample *out);
```

### MPU6050

`mpu6050_read_sample()` returns acceleration in `m/s2` and angular velocity in `deg/s`, already converted from the selected register ranges. The sample timestamp is assigned by the acquisition layer at the 25 Hz deadline. A communication or conversion failure produces an IMU fault event with the relevant result code.

### GPS PA6H

The UART receive path feeds NMEA bytes to the parser. The application contract requires GGA-derived fix quality, satellites, latitude, longitude, altitude, and UTC when available. Latitude and longitude are decimal degrees; altitude is metres above mean sea level. A valid sentence with no fix is represented by `gps_fix = false`, `satellites = 0`, and `SENSOR_OK`.

### Si7021

`si7021_read_sample()` returns temperature in degrees Celsius and relative humidity in percent RH. Values outside the sensor's physical validity range are rejected with `SENSOR_INVALID`.

### LDR

`ldr_read_raw()` returns only the ADC sample and acquisition timestamp. B's environment layer applies the frozen dark and maximum-light calibration and publishes `light_percent` in the range `0.0` to `100.0`. Saturation or an out-of-calibration sample sets the LDR error flag.

## Error model

```c
enum sensor_result {
    SENSOR_OK = 0,
    SENSOR_NOT_READY,
    SENSOR_NO_DATA,
    SENSOR_INVALID,
    SENSOR_BUS_ERROR,
    SENSOR_TIMEOUT,
    SENSOR_OVERFLOW,
    SENSOR_CONFIG_ERROR
};
```

The result code describes the latest operation. The application converts it into persistent health and SIMU protocol flags:

| Condition | SIMU flag | Fallback |
|---|---:|---|
| MPU6050 invalid/read failure | bit 0 (`0x0001`) | Last valid IMU value, otherwise formatted zero |
| GPS no fix or invalid GPS data | bit 1 (`0x0002`) | `GF=0`, `SAT=00`, retain last valid position or zero |
| LDR invalid/saturated/out of range | bit 2 (`0x0004`) | Last valid light value, otherwise zero |
| Si7021 invalid/read failure | bit 3 (`0x0008`) | Last valid environment value, otherwise zero |
| Impact/crash-like event | bit 4 (`0x0010`) | State logic decides whether `PROBLEM` applies |
| Calibration pending/invalid | bit 5 (`0x0020`) | Keep the value invalid until calibration is available |
| Position/velocity inertial-only | bit 6 (`0x0040`) | Keep integrated estimate and document drift |
| Yaw unreliable | bit 7 (`0x0080`) | Send numeric yaw and set the flag |

Bits 8 through 15 remain zero until assigned by the project specification.

## Shared telemetry model

The application publishes one coherent snapshot. Producers update their own input data, the processing task updates derived data and health, and the output task reads a copied snapshot while holding the mutex only for the copy operation.

The canonical C representation is in `include/telemetry_snapshot.h`.

Key semantics:

- `ax`, `ay`, `az` are acceleration in `m/s2`.
- `gx`, `gy`, `gz` are angular velocity in `deg/s`.
- `roll`, `pitch`, and `yaw` are degrees, initialized to zero and updated from valid gyro samples.
- `latitude` and `longitude` are decimal degrees; `altitude` is metres.
- `x`, `y`, `z` are local metres with `X=East`, `Y=North`, and `Z=Up`.
- `vx`, `vy`, `vz` are local `m/s` velocities.
- The first valid GPS fix defines the local origin; `local_origin_valid` records whether it exists.
- `light_percent` is calibrated `0.0` to `100.0`; temperature is degrees Celsius; humidity is percent RH.
- `state` is exactly one of `IDLE`, `FLYING`, or `PROBLEM`.
- `flags` is the aggregated 16-bit SIMU status mask.

## IMU event contract

The static IMU queue carries one event for every 25 Hz period:

- `IMU_EVENT_SAMPLE` contains a valid normalized sample and its timestamp.
- `IMU_EVENT_FAULT` contains the result code and timestamp for a failed period.

For `IMU_EVENT_SAMPLE`, `result` is `SENSOR_OK`. For `IMU_EVENT_FAULT`, `result` identifies the failed operation and the sample payload is ignored.

Processing must consume both event types so that health, state, and error flags continue to update when a read fails. Queue overflow is an application fault and must be recorded rather than silently dropping the event.

## Synchronization and publication

- The IMU queue is static and bounded.
- The shared snapshot is protected by a static mutex.
- Sensor I/O is never performed while holding the snapshot mutex.
- Producers copy data into the snapshot under the mutex and release it immediately.
- `output_thread` copies one complete snapshot before formatting a report or SIMU frame.
- The output layer never reads individual producer-owned fields without the snapshot contract.

## Phase 2 exit criteria

- A and B can implement against the same sensor result codes, physical units, timestamps, and validity rules.
- GPS no-fix is distinguishable from GPS communication failure.
- All SIMU flags and fallback values are mapped to a defined condition.
- The shared snapshot and IMU event types are stable enough for Phase 3 application scaffolding.
- Calibration ownership, local-coordinate semantics, and synchronization rules are documented.

