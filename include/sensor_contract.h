#ifndef SENSOR_CONTRACT_H
#define SENSOR_CONTRACT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "telemetry_snapshot.h"

#define MPU6050_ACCEL_LSB_PER_G 16384.0f
#define MPU6050_GYRO_LSB_PER_DPS 131.0f
#define MPU6050_STANDARD_GRAVITY 9.80665f

struct mpu6050_raw_sample {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

struct si7021_raw_sample {
    uint16_t temperature_raw;
    uint16_t humidity_raw;
};

struct ldr_calibration {
    uint16_t dark_raw;
    uint16_t maximum_raw;
};

enum sensor_result mpu6050_convert_raw(const struct mpu6050_raw_sample *raw,
                                        uint32_t timestamp_ms,
                                        struct imu_sample *out);

enum sensor_result si7021_convert_raw(const struct si7021_raw_sample *raw,
                                      uint32_t timestamp_ms,
                                      struct environment_sample *out);

enum sensor_result ldr_map_percent(uint16_t adc_raw,
                                   uint32_t timestamp_ms,
                                   const struct ldr_calibration *calibration,
                                   struct ldr_sample *out,
                                   float *light_percent);

enum sensor_result gps_parse_gga(const char *line, struct gps_sample *out);

enum sensor_result sensor_mock_read_imu(uint32_t timestamp_ms,
                                        struct imu_sample *out);
enum sensor_result sensor_mock_read_gps(uint32_t timestamp_ms,
                                        struct gps_sample *out);
enum sensor_result sensor_mock_read_environment(uint32_t timestamp_ms,
                                                struct environment_sample *out);
enum sensor_result sensor_mock_read_ldr(uint32_t timestamp_ms,
                                        struct ldr_sample *out);

#endif /* SENSOR_CONTRACT_H */
