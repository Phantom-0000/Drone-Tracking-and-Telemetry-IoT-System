#ifndef TELEMETRY_SNAPSHOT_H
#define TELEMETRY_SNAPSHOT_H

/*
 * Shared telemetry data contract exchanged between acquisition threads,
 * the processing pipeline, mode management, and the VCOM output thread.
 * Access to the global snapshot instance is serialized via snapshot_mutex.
 */

#include <stdbool.h>
#include <stdint.h>

/* High-level drone tracker operating states (PROBLEM takes precedence). */
enum tracker_state {
    TRACKER_STATE_IDLE = 0,
    TRACKER_STATE_FLYING,
    TRACKER_STATE_PROBLEM,
};

/* Unified status codes returned by low-level and mock sensor drivers. */
enum sensor_result {
    SENSOR_OK = 0,
    SENSOR_NOT_READY,
    SENSOR_NO_DATA,
    SENSOR_INVALID,
    SENSOR_BUS_ERROR,
    SENSOR_TIMEOUT,
    SENSOR_OVERFLOW,
    SENSOR_CONFIG_ERROR,
};

/*
 * SIMU protocol 16-bit status bitmask (Table 5 of project specification).
 * Bits 8..15 are reserved and kept at zero in transmitted SIMU frames.
 */
enum telemetry_flag {
    TELEMETRY_FLAG_IMU_INVALID = 0x0001,         /* Bit 0: MPU6050 data invalid or I2C error */
    TELEMETRY_FLAG_GPS_INVALID = 0x0002,         /* Bit 1: GPS fix unavailable or NMEA invalid */
    TELEMETRY_FLAG_LDR_INVALID = 0x0004,         /* Bit 2: LDR fault, rail, or outside calibrated range */
    TELEMETRY_FLAG_SI7021_INVALID = 0x0008,      /* Bit 3: Si7021 data invalid or I2C error */
    TELEMETRY_FLAG_IMPACT = 0x0010,              /* Bit 4: Crash-like severe tilt / inversion detected */
    TELEMETRY_FLAG_CALIBRATION_PENDING = 0x0020, /* Bit 5: Start-up IMU still calibration in progress */
    TELEMETRY_FLAG_INERTIAL_ONLY = 0x0040,       /* Bit 6: Local motion integrated from IMU without GPS */
    TELEMETRY_FLAG_YAW_UNRELIABLE = 0x0080,      /* Bit 7: Gyro-only yaw lacks absolute heading reference */
};

/* Discriminator for items delivered through imu_event_queue at 25 Hz. */
enum imu_event_kind {
    IMU_EVENT_SAMPLE = 0,
    IMU_EVENT_FAULT,
};

/* Converted 6-axis IMU measurement in physical units (m/s^2 and deg/s). */
struct imu_sample {
    uint32_t timestamp_ms;
    float ax_mps2;
    float ay_mps2;
    float az_mps2;
    float gx_dps;
    float gy_dps;
    float gz_dps;
};

/* Parsed NMEA GGA navigation and UTC time record from the PA6H GPS module. */
struct gps_sample {
    uint32_t timestamp_ms;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    uint8_t satellites;
    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_second;
    bool fix_valid;
    bool utc_valid;
};

/* Converted Si7021 environmental sample (degrees Celsius and %RH). */
struct environment_sample {
    uint32_t timestamp_ms;
    float temperature_c;
    float humidity_percent;
};

/* Raw 12-bit ADC sample (0..4095) read from the LDR voltage divider. */
struct ldr_sample {
    uint32_t timestamp_ms;
    uint16_t adc_raw;
};

/* Message queued from imu_thread to processing_thread every 40 ms (25 Hz). */
struct imu_event {
    enum imu_event_kind kind;
    enum sensor_result result;
    struct imu_sample sample;
};

/* Summary statistics computed over each completed 30-second rolling window. */
struct telemetry_statistics {
    uint32_t window_end_ms;
    /*
     * Per-channel sample counts for the completed window. A channel with a
     * zero count has no valid data in the window and its mean/min/max are
     * meaningless; the other channels are still reported.
     */
    uint32_t imu_samples;
    uint32_t light_samples;
    uint32_t environment_samples;
    float light_mean;
    float light_min;
    float light_max;
    float temperature_mean;
    float temperature_min;
    float temperature_max;
    float humidity_mean;
    float humidity_min;
    float humidity_max;
    float acceleration_mean;
    float acceleration_min;
    float acceleration_max;
    float angular_velocity_mean;
    float angular_velocity_min;
    float angular_velocity_max;
    bool valid;
};

/* Coherent system-wide telemetry state read by output_thread for VCOM output. */
struct telemetry_snapshot {
    uint32_t timestamp_ms;

    /* Latest bias-corrected IMU linear acceleration and angular velocity. */
    float ax_mps2;
    float ay_mps2;
    float az_mps2;
    float gx_dps;
    float gy_dps;
    float gz_dps;

    /* Integrated Euler attitude angles wrapped to [-180, +180) degrees. */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    /* Global GNSS position and UTC time (last valid coordinates retained on fix loss). */
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    uint32_t gps_timestamp_ms;
    uint8_t satellites;
    uint8_t gps_utc_hour;
    uint8_t gps_utc_minute;
    uint8_t gps_utc_second;
    bool gps_fix;
    bool gps_utc_valid;

    /* Calibrated ambient light (0..100 %) and Si7021 environmental readings. */
    float light_percent;
    float temperature_c;
    float humidity_percent;
    /* Incremented on every fresh measurement; used by the 30 s statistics. */
    uint32_t light_sample_seq;
    uint32_t environment_sample_seq;
    /*
     * Persistent LDR ADC failure or open/shorted divider rail (0 / 4095).
     * Unlike an out-of-calibration reading (FLAGS bit 2 only), this is a
     * device fault and counts towards the PROBLEM state.
     */
    bool ldr_hardware_fault;

    /* Local East-North-Up (ENU) position and velocity relative to origin. */
    bool local_origin_valid;
    float x_m;
    float y_m;
    float z_m;
    float vx_mps;
    float vy_mps;
    float vz_mps;

    enum tracker_state state;
    uint16_t flags;
    struct telemetry_statistics statistics;
};

#endif /* TELEMETRY_SNAPSHOT_H */
