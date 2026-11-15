#ifndef SENSOR_CONTRACT_H
#define SENSOR_CONTRACT_H

/*
 * Low-level sensor driver interface and unit-conversion contract for
 * MPU6050 (I2C1), Si7021 (I2C1), PA6H GPS (USART1), and LDR divider (ADC1).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "telemetry_snapshot.h"

/* MPU6050 sensitivity scale factors for AFS_SEL=0 (+/-2g) and FS_SEL=0 (+/-250 dps). */
#define MPU6050_ACCEL_LSB_PER_G 16384.0f
#define MPU6050_GYRO_LSB_PER_DPS 131.0f
#define MPU6050_STANDARD_GRAVITY 9.80665f

/* Raw 16-bit signed register values burst-read from MPU6050 (0x3B..0x48). */
struct mpu6050_raw_sample {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

/* Raw 16-bit measurement words read from Si7021 (status bits masked out). */
struct si7021_raw_sample {
    uint16_t temperature_raw;
    uint16_t humidity_raw;
};

/* Calibrated 12-bit ADC endpoints corresponding to 0 % (dark) and 100 % (bright). */
struct ldr_calibration {
    uint16_t dark_raw;
    uint16_t maximum_raw;
};

/* Convert raw MPU6050 counts into m/s^2 and deg/s. */
enum sensor_result mpu6050_convert_raw(const struct mpu6050_raw_sample *raw,
                                        uint32_t timestamp_ms,
                                        struct imu_sample *out);

/* Convert raw Si7021 words into degrees Celsius and clamped 0..100 %RH. */
enum sensor_result si7021_convert_raw(const struct si7021_raw_sample *raw,
                                      uint32_t timestamp_ms,
                                      struct environment_sample *out);

/*
 * Map a raw LDR ADC sample to 0.0-100.0 %.
 * SENSOR_OK:       inside the calibrated [dark_raw, maximum_raw] range.
 * SENSOR_OVERFLOW: outside the calibrated range (1..dark_raw-1 or
 *                  maximum_raw+1..4094); *light_percent holds the clamped
 *                  0 % or 100 % value and the caller sets FLAGS bit 2.
 * SENSOR_INVALID:  invalid arguments/calibration or ADC rail (0 / >=4095,
 *                  indicating an open or shorted divider; no value produced).
 */
enum sensor_result ldr_map_percent(uint16_t adc_raw,
                                   uint32_t timestamp_ms,
                                   const struct ldr_calibration *calibration,
                                   struct ldr_sample *out,
                                   float *light_percent);

/* Validate NMEA XOR checksum and parse a $GPGGA or $GNGGA sentence. */
enum sensor_result gps_parse_gga(const char *line, struct gps_sample *out);

/* Deterministic mock backends used only when CONFIG_APP_USE_MOCK_SENSORS=y. */
enum sensor_result sensor_mock_read_imu(uint32_t timestamp_ms,
                                        struct imu_sample *out);
enum sensor_result sensor_mock_read_gps(uint32_t timestamp_ms,
                                        struct gps_sample *out);
enum sensor_result sensor_mock_read_environment(uint32_t timestamp_ms,
                                                struct environment_sample *out);
enum sensor_result sensor_mock_read_ldr(uint32_t timestamp_ms,
                                        struct ldr_sample *out);

/* Register-level hardware acquisition backends for STM32WL55JC peripherals. */
enum sensor_result sensor_hw_read_imu(uint32_t timestamp_ms,
                                      struct imu_sample *out);
enum sensor_result sensor_hw_read_gps(uint32_t timestamp_ms,
                                      struct gps_sample *out);
enum sensor_result sensor_hw_read_environment(uint32_t timestamp_ms,
                                              struct environment_sample *out);
enum sensor_result sensor_hw_read_ldr(uint32_t timestamp_ms,
                                      struct ldr_sample *out);

#endif /* SENSOR_CONTRACT_H */
