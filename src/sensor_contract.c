#include "sensor_contract.h"

#include <stdlib.h>
#include <string.h>

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

enum sensor_result mpu6050_convert_raw(const struct mpu6050_raw_sample *raw,
                                        uint32_t timestamp_ms,
                                        struct imu_sample *out)
{
    if (raw == NULL || out == NULL) {
        return SENSOR_INVALID;
    }

    out->timestamp_ms = timestamp_ms;
    out->ax_mps2 = ((float)raw->accel_x / MPU6050_ACCEL_LSB_PER_G) *
                   MPU6050_STANDARD_GRAVITY;
    out->ay_mps2 = ((float)raw->accel_y / MPU6050_ACCEL_LSB_PER_G) *
                   MPU6050_STANDARD_GRAVITY;
    out->az_mps2 = ((float)raw->accel_z / MPU6050_ACCEL_LSB_PER_G) *
                   MPU6050_STANDARD_GRAVITY;
    out->gx_dps = (float)raw->gyro_x / MPU6050_GYRO_LSB_PER_DPS;
    out->gy_dps = (float)raw->gyro_y / MPU6050_GYRO_LSB_PER_DPS;
    out->gz_dps = (float)raw->gyro_z / MPU6050_GYRO_LSB_PER_DPS;
    return SENSOR_OK;
}

enum sensor_result si7021_convert_raw(const struct si7021_raw_sample *raw,
                                      uint32_t timestamp_ms,
                                      struct environment_sample *out)
{
    float temperature;
    float humidity;

    if (raw == NULL || out == NULL) {
        return SENSOR_INVALID;
    }

    temperature = (175.72f * (float)raw->temperature_raw / 65536.0f) - 46.85f;
    humidity = (125.0f * (float)raw->humidity_raw / 65536.0f) - 6.0f;
    if (temperature < -46.85f || temperature > 128.0f ||
        humidity < 0.0f || humidity > 100.0f) {
        return SENSOR_INVALID;
    }

    out->timestamp_ms = timestamp_ms;
    out->temperature_c = temperature;
    out->humidity_percent = humidity;
    return SENSOR_OK;
}

enum sensor_result ldr_map_percent(uint16_t adc_raw,
                                   uint32_t timestamp_ms,
                                   const struct ldr_calibration *calibration,
                                   struct ldr_sample *out,
                                   float *light_percent)
{
    float mapped;

    if (calibration == NULL || out == NULL || light_percent == NULL ||
        calibration->maximum_raw <= calibration->dark_raw) {
        return SENSOR_INVALID;
    }

    out->timestamp_ms = timestamp_ms;
    out->adc_raw = adc_raw;
    if (adc_raw <= calibration->dark_raw) {
        mapped = 0.0f;
    } else if (adc_raw >= calibration->maximum_raw) {
        mapped = 100.0f;
    } else {
        mapped = 100.0f * (float)(adc_raw - calibration->dark_raw) /
                 (float)(calibration->maximum_raw - calibration->dark_raw);
    }

    *light_percent = clamp_float(mapped, 0.0f, 100.0f);
    return SENSOR_OK;
}

static bool next_field(char **cursor, char *field, size_t field_size)
{
    char *start;
    char *separator;
    size_t length;

    if (cursor == NULL || *cursor == NULL || field == NULL || field_size == 0U) {
        return false;
    }

    start = *cursor;
    separator = strchr(start, ',');
    length = separator == NULL ? strlen(start) : (size_t)(separator - start);
    if (length >= field_size) {
        return false;
    }

    memcpy(field, start, length);
    field[length] = '\0';
    *cursor = separator == NULL ? NULL : separator + 1;
    return true;
}

static bool parse_nmea_coordinate(const char *value,
                                  const char *hemisphere,
                                  bool latitude,
                                  double *degrees)
{
    double raw;
    double whole_degrees;
    double minutes;

    if (value == NULL || hemisphere == NULL || degrees == NULL || value[0] == '\0') {
        return false;
    }

    raw = strtod(value, NULL);
    whole_degrees = latitude ? (double)((int)(raw / 100.0))
                             : (double)((int)(raw / 100.0));
    minutes = raw - (whole_degrees * 100.0);
    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }

    *degrees = whole_degrees + minutes / 60.0;
    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
        *degrees = -*degrees;
    } else if (hemisphere[0] != 'N' && hemisphere[0] != 'E') {
        return false;
    }
    return true;
}

enum sensor_result gps_parse_gga(const char *line, struct gps_sample *out)
{
    char buffer[160];
    char field[24];
    char *cursor;
    char *end;
    int field_index = 0;
    int fix_quality;
    long satellites;
    double latitude;
    double longitude;
    char time_field[24] = {0};
    char latitude_field[24] = {0};
    char latitude_hemisphere[4] = {0};
    char longitude_field[24] = {0};
    char longitude_hemisphere[4] = {0};
    char fix_field[24] = {0};
    char satellite_field[24] = {0};
    char altitude_field[24] = {0};

    if (line == NULL || out == NULL || strlen(line) >= sizeof(buffer)) {
        return SENSOR_INVALID;
    }

    strcpy(buffer, line);
    end = strchr(buffer, '*');
    if (end != NULL) {
        *end = '\0';
    }

    cursor = buffer;
    while (next_field(&cursor, field, sizeof(field))) {
        switch (field_index) {
        case 0:
            if (strcmp(field, "$GPGGA") != 0 && strcmp(field, "$GNGGA") != 0) {
                return SENSOR_INVALID;
            }
            break;
        case 1:
            strncpy(time_field, field, sizeof(time_field) - 1U);
            break;
        case 2:
            strncpy(latitude_field, field, sizeof(latitude_field) - 1U);
            break;
        case 3:
            strncpy(latitude_hemisphere, field, sizeof(latitude_hemisphere) - 1U);
            break;
        case 4:
            strncpy(longitude_field, field, sizeof(longitude_field) - 1U);
            break;
        case 5:
            strncpy(longitude_hemisphere, field, sizeof(longitude_hemisphere) - 1U);
            break;
        case 6:
            strncpy(fix_field, field, sizeof(fix_field) - 1U);
            break;
        case 7:
            strncpy(satellite_field, field, sizeof(satellite_field) - 1U);
            break;
        case 9:
            strncpy(altitude_field, field, sizeof(altitude_field) - 1U);
            break;
        default:
            break;
        }
        field_index++;
    }

    if (field_index < 10 || fix_field[0] == '\0' || satellite_field[0] == '\0') {
        return SENSOR_INVALID;
    }

    fix_quality = (int)strtol(fix_field, NULL, 10);
    satellites = strtol(satellite_field, NULL, 10);
    if (satellites < 0L || satellites > 99L) {
        return SENSOR_INVALID;
    }

    memset(out, 0, sizeof(*out));
    out->satellites = (uint8_t)satellites;
    out->fix_valid = fix_quality > 0;
    if (time_field[0] != '\0' && strlen(time_field) >= 6U) {
        out->utc_hour = (uint8_t)((time_field[0] - '0') * 10 + time_field[1] - '0');
        out->utc_minute = (uint8_t)((time_field[2] - '0') * 10 + time_field[3] - '0');
        out->utc_second = (uint8_t)((time_field[4] - '0') * 10 + time_field[5] - '0');
        out->utc_valid = out->utc_hour < 24U && out->utc_minute < 60U &&
                         out->utc_second < 60U;
    }

    if (!out->fix_valid) {
        return SENSOR_OK;
    }
    if (!parse_nmea_coordinate(latitude_field, latitude_hemisphere, true, &latitude) ||
        !parse_nmea_coordinate(longitude_field, longitude_hemisphere, false, &longitude)) {
        return SENSOR_INVALID;
    }

    out->latitude_deg = latitude;
    out->longitude_deg = longitude;
    out->altitude_m = (float)strtod(altitude_field, NULL);
    return SENSOR_OK;
}

enum sensor_result sensor_mock_read_imu(uint32_t timestamp_ms,
                                        struct imu_sample *out)
{
    struct mpu6050_raw_sample raw = {
        .accel_z = (int16_t)MPU6050_ACCEL_LSB_PER_G,
    };

    return mpu6050_convert_raw(&raw, timestamp_ms, out);
}

enum sensor_result sensor_mock_read_gps(uint32_t timestamp_ms,
                                        struct gps_sample *out)
{
    static const char gga[] =
        "$GPGGA,123456.00,4038.9462,N,00337.6450,W,1,08,1.0,628.4,M,0.0,M,,*00";
    enum sensor_result result = gps_parse_gga(gga, out);

    if (result == SENSOR_OK && out != NULL) {
        out->timestamp_ms = timestamp_ms;
    }
    return result;
}

enum sensor_result sensor_mock_read_environment(uint32_t timestamp_ms,
                                                struct environment_sample *out)
{
    struct si7021_raw_sample raw = {
        .temperature_raw = 26400U,
        .humidity_raw = 36000U,
    };

    return si7021_convert_raw(&raw, timestamp_ms, out);
}

enum sensor_result sensor_mock_read_ldr(uint32_t timestamp_ms,
                                        struct ldr_sample *out)
{
    if (out == NULL) {
        return SENSOR_INVALID;
    }
    out->timestamp_ms = timestamp_ms;
    out->adc_raw = 500U;
    return SENSOR_OK;
}
