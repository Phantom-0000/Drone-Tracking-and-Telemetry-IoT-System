#ifndef TELEMETRY_SNAPSHOT_H
#define TELEMETRY_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

enum tracker_state {
    TRACKER_STATE_IDLE = 0,
    TRACKER_STATE_FLYING,
    TRACKER_STATE_PROBLEM,
};

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

enum telemetry_flag {
    TELEMETRY_FLAG_IMU_INVALID = 0x0001,
    TELEMETRY_FLAG_GPS_INVALID = 0x0002,
    TELEMETRY_FLAG_LDR_INVALID = 0x0004,
    TELEMETRY_FLAG_SI7021_INVALID = 0x0008,
    TELEMETRY_FLAG_IMPACT = 0x0010,
    TELEMETRY_FLAG_CALIBRATION_PENDING = 0x0020,
    TELEMETRY_FLAG_INERTIAL_ONLY = 0x0040,
    TELEMETRY_FLAG_YAW_UNRELIABLE = 0x0080,
};

enum imu_event_kind {
    IMU_EVENT_SAMPLE = 0,
    IMU_EVENT_FAULT,
};

struct imu_sample {
    uint32_t timestamp_ms;
    float ax_mps2;
    float ay_mps2;
    float az_mps2;
    float gx_dps;
    float gy_dps;
    float gz_dps;
};

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

struct environment_sample {
    uint32_t timestamp_ms;
    float temperature_c;
    float humidity_percent;
};

struct ldr_sample {
    uint32_t timestamp_ms;
    uint16_t adc_raw;
};

struct imu_event {
    enum imu_event_kind kind;
    enum sensor_result result;
    struct imu_sample sample;
};

struct telemetry_snapshot {
    uint32_t timestamp_ms;

    float ax_mps2;
    float ay_mps2;
    float az_mps2;
    float gx_dps;
    float gy_dps;
    float gz_dps;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    uint8_t satellites;
    uint8_t gps_utc_hour;
    uint8_t gps_utc_minute;
    uint8_t gps_utc_second;
    bool gps_fix;
    bool gps_utc_valid;

    float light_percent;
    float temperature_c;
    float humidity_percent;

    bool local_origin_valid;
    float x_m;
    float y_m;
    float z_m;
    float vx_mps;
    float vy_mps;
    float vz_mps;

    enum tracker_state state;
    uint16_t flags;
};

#endif /* TELEMETRY_SNAPSHOT_H */
