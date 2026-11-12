#include "sensor_contract.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef __ZEPHYR__
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#endif

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

    if (raw->temperature_raw == 0U || raw->temperature_raw >= 0xFFF0U ||
        raw->humidity_raw == 0U || raw->humidity_raw >= 0xFFF0U) {
        return SENSOR_INVALID;
    }

    temperature = (175.72f * (float)raw->temperature_raw / 65536.0f) - 46.85f;
    humidity = (125.0f * (float)raw->humidity_raw / 65536.0f) - 6.0f;
    if (temperature < -45.0f || temperature > 128.0f ||
        humidity < -6.0f || humidity > 118.0f) {
        return SENSOR_INVALID;
    }

    out->timestamp_ms = timestamp_ms;
    out->temperature_c = temperature;
    out->humidity_percent = clamp_float(humidity, 0.0f, 100.0f);
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
    if (adc_raw == 0U || adc_raw >= 4095U) {
        /* ADC rail: open or shorted voltage divider fault. */
        return SENSOR_INVALID;
    }
    if (adc_raw < calibration->dark_raw) {
        /* Darker than the calibrated 0 % point. */
        *light_percent = 0.0f;
        return SENSOR_OVERFLOW;
    }
    if (adc_raw > calibration->maximum_raw) {
        /* Brighter than the calibrated 100 % point. */
        *light_percent = 100.0f;
        return SENSOR_OVERFLOW;
    }

    mapped = 100.0f * (float)(adc_raw - calibration->dark_raw) /
             (float)(calibration->maximum_raw - calibration->dark_raw);
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

static int hex_digit_value(char value)
{
    if (value >= '0' && value <= '9') {
        return value - '0';
    }
    if (value >= 'A' && value <= 'F') {
        return value - 'A' + 10;
    }
    if (value >= 'a' && value <= 'f') {
        return value - 'a' + 10;
    }
    return -1;
}

static bool nmea_checksum_valid(const char *line, const char *checksum_marker)
{
    unsigned int checksum = 0U;
    int high;
    int low;

    if (line == NULL || checksum_marker == NULL || checksum_marker <= line ||
        line[0] != '$') {
        return false;
    }
    for (const char *cursor = line + 1; cursor < checksum_marker; ++cursor) {
        checksum ^= (unsigned char)*cursor;
    }
    if (checksum_marker[1] == '\0' || checksum_marker[2] == '\0') {
        return false;
    }
    high = hex_digit_value(checksum_marker[1]);
    low = hex_digit_value(checksum_marker[2]);
    if (high < 0 || low < 0) {
        return false;
    }
    if (checksum_marker[3] != '\0' && checksum_marker[3] != '\r' &&
        checksum_marker[3] != '\n') {
        return false;
    }
    return checksum == (unsigned int)((high << 4) | low);
}

static bool parse_long_strict(const char *text, long *value)
{
    char *end;
    long parsed;

    if (text == NULL || value == NULL || text[0] == '\0') {
        return false;
    }
    errno = 0;
    parsed = strtol(text, &end, 10);
    if (errno == ERANGE || end == text || *end != '\0') {
        return false;
    }
    *value = parsed;
    return true;
}

static bool parse_double_strict(const char *text, double *value)
{
    char *end;
    double parsed;

    if (text == NULL || value == NULL || text[0] == '\0') {
        return false;
    }
    errno = 0;
    parsed = strtod(text, &end);
    if (errno == ERANGE || end == text || *end != '\0' ||
        !isfinite(parsed)) {
        return false;
    }
    *value = parsed;
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

    if (!parse_double_strict(value, &raw) || raw < 0.0) {
        return false;
    }
    whole_degrees = floor(raw / 100.0);
    minutes = raw - (whole_degrees * 100.0);
    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }
    if (whole_degrees > (latitude ? 90.0 : 180.0) ||
        (whole_degrees == (latitude ? 90.0 : 180.0) && minutes != 0.0)) {
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
    long fix_quality;
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
    if (!nmea_checksum_valid(buffer, end)) {
        return SENSOR_INVALID;
    }
    *end = '\0';

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

    if (!parse_long_strict(fix_field, &fix_quality) || fix_quality < 0L ||
        !parse_long_strict(satellite_field, &satellites) || satellites < 0L ||
        satellites > 99L) {
        return SENSOR_INVALID;
    }

    memset(out, 0, sizeof(*out));
    out->fix_valid = fix_quality > 0;
    if (time_field[0] != '\0' && strlen(time_field) >= 6U) {
        for (size_t index = 0U; index < 6U; ++index) {
            if (time_field[index] < '0' || time_field[index] > '9') {
                return SENSOR_INVALID;
            }
        }
        out->utc_hour = (uint8_t)((time_field[0] - '0') * 10 + time_field[1] - '0');
        out->utc_minute = (uint8_t)((time_field[2] - '0') * 10 + time_field[3] - '0');
        out->utc_second = (uint8_t)((time_field[4] - '0') * 10 + time_field[5] - '0');
        out->utc_valid = out->utc_hour < 24U && out->utc_minute < 60U &&
                         out->utc_second < 60U;
    }

    if (!out->fix_valid) {
        out->satellites = 0U;
        return SENSOR_OK;
    }
    if (!parse_nmea_coordinate(latitude_field, latitude_hemisphere, true, &latitude) ||
        !parse_nmea_coordinate(longitude_field, longitude_hemisphere, false, &longitude)) {
        return SENSOR_INVALID;
    }

    out->latitude_deg = latitude;
    out->longitude_deg = longitude;
    {
        double altitude;

        if (!parse_double_strict(altitude_field, &altitude)) {
            return SENSOR_INVALID;
        }
        out->altitude_m = (float)altitude;
    }
    out->satellites = (uint8_t)satellites;
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
        "$GPGGA,123456.00,4038.9462,N,00337.6450,W,1,08,1.0,628.4,M,0.0,M,,*4E";
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

#ifdef __ZEPHYR__

/*
 * Explicit hardware register definitions (Course Guideline Section 2.8:
 * direct register-level bus transactions over black-box high-level drivers).
 */
#define MPU6050_I2C_ADDR            0x68U
#define MPU6050_REG_CONFIG          0x1AU /* DLPF_CFG */
#define MPU6050_REG_GYRO_CONFIG     0x1BU /* FS_SEL = 0 -> +/-250 dps */
#define MPU6050_REG_ACCEL_CONFIG    0x1CU /* AFS_SEL = 0 -> +/-2g */
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU /* 14-byte burst: Accel(6)+Temp(2)+Gyro(6) */
#define MPU6050_REG_PWR_MGMT_1      0x6BU
/*
 * PWR_MGMT_1 = 0x09: SLEEP = 0 (wake), TEMP_DIS = 1 (die temperature sensor
 * unused, powered down), CLKSEL = 1 (PLL with gyro X reference, more stable
 * than the internal 8 MHz oscillator recommended by the datasheet).
 */
#define MPU6050_PWR_MGMT_1_VALUE    0x09U
/*
 * CONFIG = 0x05: DLPF_CFG = 5 -> accel 10 Hz / gyro 10 Hz bandwidth. The
 * application samples at 25 Hz (Nyquist 12.5 Hz), so this is the anti-alias
 * filter and also reduces the noise seen by the state detector.
 */
#define MPU6050_CONFIG_VALUE        0x05U

#define SI7021_I2C_ADDR             0x40U
#define SI7021_CMD_MEAS_RH_NO_HOLD  0xF5U /* Measure RH, No Hold Master Mode */
#define SI7021_CMD_READ_PREV_TEMP   0xE0U /* Read Temp from Previous RH Measurement */

#define GPS_STALE_TIMEOUT_MS        3000U
/*
 * PMTK314: enable only GGA (1 per fix) and disable RMC/GSA/GSV/VTG/GLL. The
 * application parses GGA only, so the other sentences would just wake the
 * MCU with UART interrupts. The setting is volatile in the PA6H, so it is
 * re-sent if other NMEA sentences are observed again (e.g. GPS power cycle).
 */
#define GPS_PMTK_GGA_ONLY \
    "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define GPS_PMTK_RETRY_MS           10000U

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
static const struct device *const i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
static K_MUTEX_DEFINE(i2c1_bus_mutex);
static bool mpu6050_initialized;
#endif

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
static const struct adc_dt_spec ldr_adc_channel =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static bool ldr_adc_initialized;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart1), okay)
static const struct device *const gps_uart_dev =
    DEVICE_DT_GET(DT_NODELABEL(usart1));
static struct k_spinlock gps_spinlock;
static char gps_rx_work_line[160];
static size_t gps_rx_work_len;
static char gps_latest_gga_line[160];
static uint32_t gps_latest_gga_rx_ms;
static bool gps_latest_gga_unread;
static bool gps_uart_initialized;
static struct gps_sample gps_cached_sample;
static uint32_t gps_cached_rx_ms;
static bool gps_cached_valid;
/* Set by the ISR when a non-GGA NMEA sentence ($GP.../$GN...) is received. */
static atomic_t gps_other_sentence_seen;
static uint32_t gps_pmtk_last_ms;
static bool gps_pmtk_sent;

static void gps_send_command(const char *command)
{
    /*
     * Polled TX on USART1 (~55 ms at 9600 baud). Only used at start-up and
     * when the GPS has lost the volatile PMTK314 setting, never periodically.
     */
    for (const char *cursor = command; *cursor != '\0'; ++cursor) {
        uart_poll_out(gps_uart_dev, (unsigned char)*cursor);
    }
}

static void gps_uart_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (!uart_irq_rx_ready(dev)) {
            continue;
        }

        uint8_t buffer[32];
        int received = uart_fifo_read(dev, buffer, sizeof(buffer));

        for (int i = 0; i < received; ++i) {
            char ch = (char)buffer[i];

            if (ch == '$') {
                gps_rx_work_len = 0U;
                gps_rx_work_line[gps_rx_work_len++] = ch;
            } else if (ch == '\r' || ch == '\n') {
                if (gps_rx_work_len >= 7U) {
                    gps_rx_work_line[gps_rx_work_len] = '\0';
                    if (strncmp(gps_rx_work_line, "$GPGGA,", 7) == 0 ||
                        strncmp(gps_rx_work_line, "$GNGGA,", 7) == 0) {
                        k_spinlock_key_t key = k_spin_lock(&gps_spinlock);

                        memcpy(gps_latest_gga_line, gps_rx_work_line,
                               gps_rx_work_len + 1U);
                        gps_latest_gga_rx_ms = k_uptime_get_32();
                        gps_latest_gga_unread = true;
                        k_spin_unlock(&gps_spinlock, key);
                    } else if (strncmp(gps_rx_work_line, "$GP", 3) == 0 ||
                               strncmp(gps_rx_work_line, "$GN", 3) == 0) {
                        /* PMTK acknowledgements ($PMTK...) are ignored. */
                        atomic_set(&gps_other_sentence_seen, 1);
                    }
                }
                gps_rx_work_len = 0U;
            } else if (gps_rx_work_len > 0U) {
                if (gps_rx_work_len < (sizeof(gps_rx_work_line) - 1U)) {
                    gps_rx_work_line[gps_rx_work_len++] = ch;
                } else {
                    gps_rx_work_len = 0U;
                }
            }
        }
    }
}
#endif

enum sensor_result sensor_hw_read_imu(uint32_t timestamp_ms,
                                      struct imu_sample *out)
{
    if (out == NULL) {
        return SENSOR_INVALID;
    }

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
    if (!device_is_ready(i2c1_dev)) {
        return SENSOR_NOT_READY;
    }

    uint8_t start_reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t raw_bytes[14] = {0};
    struct mpu6050_raw_sample raw;
    int rc;

    for (int try_wake = 0; try_wake < 2; ++try_wake) {
        if (!mpu6050_initialized) {
            static const uint8_t wake_cmd[2] = {MPU6050_REG_PWR_MGMT_1,
                                                MPU6050_PWR_MGMT_1_VALUE};
            static const uint8_t dlpf_cfg[2] = {MPU6050_REG_CONFIG,
                                                MPU6050_CONFIG_VALUE};
            static const uint8_t gyro_cfg[2] = {MPU6050_REG_GYRO_CONFIG, 0x00U};
            static const uint8_t accel_cfg[2] = {MPU6050_REG_ACCEL_CONFIG, 0x00U};

            k_mutex_lock(&i2c1_bus_mutex, K_FOREVER);
            rc = i2c_write(i2c1_dev, wake_cmd, sizeof(wake_cmd),
                           MPU6050_I2C_ADDR);
            if (rc == 0) {
                rc = i2c_write(i2c1_dev, dlpf_cfg, sizeof(dlpf_cfg),
                               MPU6050_I2C_ADDR);
            }
            if (rc == 0) {
                rc = i2c_write(i2c1_dev, gyro_cfg, sizeof(gyro_cfg),
                               MPU6050_I2C_ADDR);
            }
            if (rc == 0) {
                rc = i2c_write(i2c1_dev, accel_cfg, sizeof(accel_cfg),
                               MPU6050_I2C_ADDR);
            }
            k_mutex_unlock(&i2c1_bus_mutex);

            if (rc != 0) {
                return SENSOR_BUS_ERROR;
            }
            mpu6050_initialized = true;
            k_msleep(35);
        }

        k_mutex_lock(&i2c1_bus_mutex, K_FOREVER);
        rc = i2c_write_read(i2c1_dev, MPU6050_I2C_ADDR, &start_reg, 1U,
                            raw_bytes, sizeof(raw_bytes));
        k_mutex_unlock(&i2c1_bus_mutex);

        if (rc != 0) {
            mpu6050_initialized = false;
            return SENSOR_BUS_ERROR;
        }

        raw.accel_x = (int16_t)(((uint16_t)raw_bytes[0] << 8) | raw_bytes[1]);
        raw.accel_y = (int16_t)(((uint16_t)raw_bytes[2] << 8) | raw_bytes[3]);
        raw.accel_z = (int16_t)(((uint16_t)raw_bytes[4] << 8) | raw_bytes[5]);
        /* raw_bytes[6..7] are die temperature (TEMP_OUT_H/L), skipped */
        raw.gyro_x = (int16_t)(((uint16_t)raw_bytes[8] << 8) | raw_bytes[9]);
        raw.gyro_y = (int16_t)(((uint16_t)raw_bytes[10] << 8) | raw_bytes[11]);
        raw.gyro_z = (int16_t)(((uint16_t)raw_bytes[12] << 8) | raw_bytes[13]);

        /*
         * If all six 16-bit accel and gyro channels read zero, the MPU6050
         * reset into sleep mode (PWR_MGMT_1 = 0x40) after a breadboard power
         * glitch; re-initialize once and re-read.
         */
        if (raw.accel_x == 0 && raw.accel_y == 0 && raw.accel_z == 0 &&
            raw.gyro_x == 0 && raw.gyro_y == 0 && raw.gyro_z == 0) {
            mpu6050_initialized = false;
            continue;
        }
        break;
    }

    if (!mpu6050_initialized) {
        return SENSOR_BUS_ERROR;
    }

    return mpu6050_convert_raw(&raw, timestamp_ms, out);
#else
    (void)timestamp_ms;
    return SENSOR_NOT_READY;
#endif
}

enum sensor_result sensor_hw_read_environment(uint32_t timestamp_ms,
                                              struct environment_sample *out)
{
    if (out == NULL) {
        return SENSOR_INVALID;
    }

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
    if (!device_is_ready(i2c1_dev)) {
        return SENSOR_NOT_READY;
    }

    uint8_t cmd_rh = SI7021_CMD_MEAS_RH_NO_HOLD;
    uint8_t cmd_prev_temp = SI7021_CMD_READ_PREV_TEMP;
    uint8_t rh_bytes[2] = {0};
    uint8_t temp_bytes[2] = {0};
    int rc = -1;

    /*
     * Hold the I2C1 bus mutex across the 30 ms conversion (once every 2 s)
     * so Si7021 has finished its max 22.8 ms RH+Temp conversion before the
     * read and never NACKs or collides with MPU6050 transfers on I2C1.
     */
    k_mutex_lock(&i2c1_bus_mutex, K_FOREVER);
    rc = i2c_write(i2c1_dev, &cmd_rh, 1U, SI7021_I2C_ADDR);
    if (rc == 0) {
        k_msleep(30);
        for (int attempt = 0; attempt < 3; ++attempt) {
            rc = i2c_read(i2c1_dev, rh_bytes, 2U, SI7021_I2C_ADDR);
            if (rc == 0) {
                break;
            }
            k_msleep(5);
        }
    }
    if (rc == 0) {
        rc = i2c_write_read(i2c1_dev, SI7021_I2C_ADDR, &cmd_prev_temp, 1U,
                            temp_bytes, 2U);
    }
    k_mutex_unlock(&i2c1_bus_mutex);

    if (rc != 0) {
        return SENSOR_BUS_ERROR;
    }

    struct si7021_raw_sample raw = {
        .humidity_raw =
            ((uint16_t)rh_bytes[0] << 8) | (uint16_t)(rh_bytes[1] & 0xFCU),
        .temperature_raw =
            ((uint16_t)temp_bytes[0] << 8) | (uint16_t)(temp_bytes[1] & 0xFCU),
    };

    return si7021_convert_raw(&raw, timestamp_ms, out);
#else
    (void)timestamp_ms;
    return SENSOR_NOT_READY;
#endif
}

enum sensor_result sensor_hw_read_ldr(uint32_t timestamp_ms,
                                      struct ldr_sample *out)
{
    if (out == NULL) {
        return SENSOR_INVALID;
    }

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
    if (!adc_is_ready_dt(&ldr_adc_channel)) {
        return SENSOR_NOT_READY;
    }

    if (!ldr_adc_initialized) {
        if (adc_channel_setup_dt(&ldr_adc_channel) < 0) {
            return SENSOR_CONFIG_ERROR;
        }
        ldr_adc_initialized = true;
    }

    int16_t sample_buffer = 0;
    struct adc_sequence sequence = {
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
    };

    if (adc_sequence_init_dt(&ldr_adc_channel, &sequence) < 0) {
        return SENSOR_CONFIG_ERROR;
    }
    if (adc_read_dt(&ldr_adc_channel, &sequence) < 0) {
        return SENSOR_BUS_ERROR;
    }

    if (sample_buffer < 0) {
        sample_buffer = 0;
    } else if (sample_buffer > 4095) {
        sample_buffer = 4095;
    }

    out->timestamp_ms = timestamp_ms;
    out->adc_raw = (uint16_t)sample_buffer;
    return SENSOR_OK;
#else
    (void)timestamp_ms;
    return SENSOR_NOT_READY;
#endif
}

enum sensor_result sensor_hw_read_gps(uint32_t timestamp_ms,
                                      struct gps_sample *out)
{
    if (out == NULL) {
        return SENSOR_INVALID;
    }

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart1), okay)
    if (!device_is_ready(gps_uart_dev)) {
        return SENSOR_NOT_READY;
    }

    if (!gps_uart_initialized) {
        uint8_t flush_byte;

        uart_irq_rx_disable(gps_uart_dev);
        while (uart_fifo_read(gps_uart_dev, &flush_byte, 1) > 0) {
        }
        if (uart_irq_callback_user_data_set(gps_uart_dev, gps_uart_isr, NULL) < 0) {
            return SENSOR_CONFIG_ERROR;
        }
        uart_irq_rx_enable(gps_uart_dev);
        gps_uart_initialized = true;
    }

    if (!gps_pmtk_sent ||
        (atomic_get(&gps_other_sentence_seen) != 0 &&
         (uint32_t)(timestamp_ms - gps_pmtk_last_ms) >= GPS_PMTK_RETRY_MS)) {
        atomic_clear(&gps_other_sentence_seen);
        gps_send_command(GPS_PMTK_GGA_ONLY);
        gps_pmtk_last_ms = timestamp_ms;
        gps_pmtk_sent = true;
    }

    char local_gga[160];
    uint32_t rx_ms = 0U;
    bool has_unread = false;

    k_spinlock_key_t key = k_spin_lock(&gps_spinlock);
    if (gps_latest_gga_unread) {
        memcpy(local_gga, gps_latest_gga_line, sizeof(local_gga));
        rx_ms = gps_latest_gga_rx_ms;
        gps_latest_gga_unread = false;
        has_unread = true;
    }
    k_spin_unlock(&gps_spinlock, key);

    if (has_unread) {
        struct gps_sample parsed;

        if (gps_parse_gga(local_gga, &parsed) == SENSOR_OK) {
            parsed.timestamp_ms = rx_ms;
            gps_cached_sample = parsed;
            gps_cached_rx_ms = rx_ms;
            gps_cached_valid = true;
        }
    }

    if (gps_cached_valid) {
        int32_t age_ms = (int32_t)(timestamp_ms - gps_cached_rx_ms);

        if (age_ms < 0) {
            age_ms = 0;
        }
        if ((uint32_t)age_ms <= GPS_STALE_TIMEOUT_MS) {
            *out = gps_cached_sample;
            return SENSOR_OK;
        }
    }

    return SENSOR_NO_DATA;
#else
    (void)timestamp_ms;
    return SENSOR_NOT_READY;
#endif
}

#else /* !__ZEPHYR__ */

enum sensor_result sensor_hw_read_imu(uint32_t timestamp_ms,
                                      struct imu_sample *out)
{
    (void)timestamp_ms;
    return out == NULL ? SENSOR_INVALID : SENSOR_NOT_READY;
}

enum sensor_result sensor_hw_read_gps(uint32_t timestamp_ms,
                                      struct gps_sample *out)
{
    (void)timestamp_ms;
    return out == NULL ? SENSOR_INVALID : SENSOR_NOT_READY;
}

enum sensor_result sensor_hw_read_environment(uint32_t timestamp_ms,
                                              struct environment_sample *out)
{
    (void)timestamp_ms;
    return out == NULL ? SENSOR_INVALID : SENSOR_NOT_READY;
}

enum sensor_result sensor_hw_read_ldr(uint32_t timestamp_ms,
                                      struct ldr_sample *out)
{
    (void)timestamp_ms;
    return out == NULL ? SENSOR_INVALID : SENSOR_NOT_READY;
}

#endif /* __ZEPHYR__ */
