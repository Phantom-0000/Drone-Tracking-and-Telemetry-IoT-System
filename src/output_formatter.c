#include "output_formatter.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static int append_bytes(char *output, size_t *used, size_t capacity,
                        const char *bytes, size_t length)
{
    if (length > capacity - *used) {
        return -ENOSPC;
    }

    memcpy(&output[*used], bytes, length);
    *used += length;
    return 0;
}

static int append_literal(char *output, size_t *used, size_t capacity,
                          const char *literal)
{
    return append_bytes(output, used, capacity, literal, strlen(literal));
}

static uint32_t decimal_scale(unsigned int decimals)
{
    uint32_t scale = 1U;

    for (unsigned int i = 0U; i < decimals; ++i) {
        scale *= 10U;
    }
    return scale;
}

static int append_uint(char *output, size_t *used, size_t capacity,
                       uint32_t value, unsigned int width)
{
    char field[12];

    if (width >= sizeof(field)) {
        return -EINVAL;
    }

    for (unsigned int i = 0U; i < width; ++i) {
        field[width - 1U - i] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    if (value != 0U) {
        return -ERANGE;
    }
    return append_bytes(output, used, capacity, field, width);
}

static int append_hex16(char *output, size_t *used, size_t capacity,
                        uint16_t value)
{
    static const char digits[] = "0123456789ABCDEF";
    char field[4] = {
        digits[(value >> 12) & 0x0FU],
        digits[(value >> 8) & 0x0FU],
        digits[(value >> 4) & 0x0FU],
        digits[value & 0x0FU],
    };

    return append_bytes(output, used, capacity, field, sizeof(field));
}

static int append_fixed(char *output, size_t *used, size_t capacity,
                        double value, bool signed_value,
                        unsigned int integer_digits,
                        unsigned int decimal_digits)
{
    char field[24];
    uint32_t scale = decimal_scale(decimal_digits);
    uint64_t maximum = 1U;
    bool negative = isfinite(value) && value < 0.0;
    double magnitude = isfinite(value) ? (negative ? -value : value) : 0.0;
    uint64_t scaled;
    unsigned int width;
    uint64_t integer_part;
    uint32_t fractional_part;

    if (!signed_value && negative) {
        magnitude = 0.0;
    }
    for (unsigned int i = 0U; i < integer_digits + decimal_digits; ++i) {
        maximum *= 10U;
    }
    maximum -= 1U;
    if (magnitude >= (double)maximum / (double)scale) {
        scaled = maximum;
    } else {
        scaled = (uint64_t)(magnitude * (double)scale + 0.5);
    }

    width = (signed_value ? 1U : 0U) + integer_digits +
            (decimal_digits != 0U ? 1U + decimal_digits : 0U);
    if (width >= sizeof(field)) {
        return -EINVAL;
    }

    unsigned int index = 0U;
    if (signed_value) {
        field[index++] = negative ? '-' : '+';
    }
    integer_part = scaled / scale;
    fractional_part = (uint32_t)(scaled % scale);
    for (unsigned int i = 0U; i < integer_digits; ++i) {
        field[index + integer_digits - 1U - i] =
            (char)('0' + (integer_part % 10U));
        integer_part /= 10U;
    }
    index += integer_digits;
    if (decimal_digits != 0U) {
        field[index++] = '.';
        for (unsigned int i = 0U; i < decimal_digits; ++i) {
            field[index + decimal_digits - 1U - i] =
                (char)('0' + (fractional_part % 10U));
            fractional_part /= 10U;
        }
    }
    return append_bytes(output, used, capacity, field, width);
}

static const char *state_text(enum tracker_state state)
{
    switch (state) {
    case TRACKER_STATE_IDLE:
        return "IDLE";
    case TRACKER_STATE_FLYING:
        return "FLYING";
    case TRACKER_STATE_PROBLEM:
        return "PROBLEM";
    default:
        return "PROBLEM";
    }
}

static int append_statistics(char *output, size_t *used, size_t capacity,
                             const struct telemetry_statistics *statistics)
{
    int rc = 0;

#define TEXT(value)                                                            \
    do {                                                                        \
        if (rc == 0) {                                                          \
            rc = append_literal(output, used, capacity, (value));              \
        }                                                                       \
    } while (0)
#define UINT(value, width)                                                      \
    do {                                                                        \
        if (rc == 0) {                                                          \
            rc = append_uint(output, used, capacity, (value), (width));         \
        }                                                                       \
    } while (0)
#define FIXED(value, signed_value, integer_digits, decimal_digits)              \
    do {                                                                        \
        if (rc == 0) {                                                          \
            rc = append_fixed(output, used, capacity, (value), (signed_value),  \
                              (integer_digits), (decimal_digits));              \
        }                                                                       \
    } while (0)

    TEXT("STATISTICS: window_end=");
    UINT(statistics->window_end_ms, 10U);
    TEXT(" imu_samples=");
    UINT(statistics->imu_samples, 5U);
    TEXT(" light_samples=");
    UINT(statistics->light_samples, 5U);
    TEXT(" env_samples=");
    UINT(statistics->environment_samples, 5U);
    TEXT("\r\n");

    /* A channel without valid samples in the window is reported as N/A. */
    TEXT("STAT LIGHT: ");
    if (statistics->light_samples == 0U) {
        TEXT("N/A");
    } else {
        TEXT("mean="); FIXED(statistics->light_mean, false, 3U, 1U);
        TEXT(" min="); FIXED(statistics->light_min, false, 3U, 1U);
        TEXT(" max="); FIXED(statistics->light_max, false, 3U, 1U);
    }
    TEXT("\r\nSTAT TEMP: ");
    if (statistics->environment_samples == 0U) {
        TEXT("N/A");
    } else {
        TEXT("mean="); FIXED(statistics->temperature_mean, true, 3U, 1U);
        TEXT(" min="); FIXED(statistics->temperature_min, true, 3U, 1U);
        TEXT(" max="); FIXED(statistics->temperature_max, true, 3U, 1U);
    }
    TEXT("\r\nSTAT HUM: ");
    if (statistics->environment_samples == 0U) {
        TEXT("N/A");
    } else {
        TEXT("mean="); FIXED(statistics->humidity_mean, false, 3U, 1U);
        TEXT(" min="); FIXED(statistics->humidity_min, false, 3U, 1U);
        TEXT(" max="); FIXED(statistics->humidity_max, false, 3U, 1U);
    }
    TEXT("\r\nSTAT ACCEL: ");
    if (statistics->imu_samples == 0U) {
        TEXT("N/A");
    } else {
        TEXT("mean="); FIXED(statistics->acceleration_mean, true, 3U, 2U);
        TEXT(" min="); FIXED(statistics->acceleration_min, true, 3U, 2U);
        TEXT(" max="); FIXED(statistics->acceleration_max, true, 3U, 2U);
    }
    TEXT("\r\nSTAT GYRO: ");
    if (statistics->imu_samples == 0U) {
        TEXT("N/A");
    } else {
        TEXT("mean="); FIXED(statistics->angular_velocity_mean, true, 4U, 1U);
        TEXT(" min="); FIXED(statistics->angular_velocity_min, true, 4U, 1U);
        TEXT(" max="); FIXED(statistics->angular_velocity_max, true, 4U, 1U);
    }
    TEXT("\r\n");

#undef FIXED
#undef UINT
#undef TEXT
    return rc;
}

static int format_report(const struct telemetry_snapshot *snapshot,
                         const char *mode, bool include_statistics,
                         char *output, size_t capacity)
{
    int rc = 0;
    size_t used = 0U;
    uint32_t satellites;
    uint32_t timestamp;

    if (snapshot == NULL || mode == NULL || output == NULL || capacity == 0U) {
        return -EINVAL;
    }
    satellites = snapshot->gps_fix ? snapshot->satellites : 0U;
    timestamp = snapshot->timestamp_ms;
    if (satellites > 99U) {
        satellites = 99U;
    }

#define TEXT(value)                                                            \
    do {                                                                        \
        if (rc == 0) {                                                          \
            rc = append_literal(output, &used, capacity, (value));             \
        }                                                                       \
    } while (0)
#define UINT(value, width)                                                      \
    do {                                                                        \
        if (rc == 0) {                                                          \
            rc = append_uint(output, &used, capacity, (value), (width));        \
        }                                                                       \
    } while (0)
#define FIXED(value, signed_value, integer_digits, decimal_digits)              \
    do {                                                                        \
        if (rc == 0) {                                                          \
            rc = append_fixed(output, &used, capacity, (value), (signed_value), \
                              (integer_digits), (decimal_digits));              \
        }                                                                       \
    } while (0)

    TEXT("MODE: "); TEXT(mode); TEXT("\r\nTIME: "); UINT(timestamp, 10U);
    TEXT(" ms\r\nGPS: fix="); UINT(snapshot->gps_fix ? 1U : 0U, 1U);
    TEXT(" sats="); UINT(satellites, 2U); TEXT(" lat=");
    FIXED(snapshot->latitude_deg, true, 2U, 6U); TEXT(" lon=");
    FIXED(snapshot->longitude_deg, true, 3U, 6U); TEXT(" alt=");
    FIXED(snapshot->altitude_m, true, 5U, 1U); TEXT(" utc=");
    UINT(snapshot->gps_utc_hour, 2U); TEXT(":"); UINT(snapshot->gps_utc_minute, 2U);
    TEXT(":"); UINT(snapshot->gps_utc_second, 2U); TEXT("\r\n");

    TEXT("IMU ACCEL: ax="); FIXED(snapshot->ax_mps2, true, 3U, 2U);
    TEXT(" ay="); FIXED(snapshot->ay_mps2, true, 3U, 2U);
    TEXT(" az="); FIXED(snapshot->az_mps2, true, 3U, 2U); TEXT("\r\n");
    TEXT("IMU GYRO: gx="); FIXED(snapshot->gx_dps, true, 4U, 1U);
    TEXT(" gy="); FIXED(snapshot->gy_dps, true, 4U, 1U);
    TEXT(" gz="); FIXED(snapshot->gz_dps, true, 4U, 1U); TEXT("\r\n");
    TEXT("ATTITUDE: roll="); FIXED(snapshot->roll_deg, true, 3U, 2U);
    TEXT(" pitch="); FIXED(snapshot->pitch_deg, true, 3U, 2U);
    TEXT(" yaw="); FIXED(snapshot->yaw_deg, true, 3U, 2U); TEXT("\r\n");
    TEXT("LOCAL: x="); FIXED(snapshot->x_m, true, 4U, 2U);
    TEXT(" y="); FIXED(snapshot->y_m, true, 4U, 2U);
    TEXT(" z="); FIXED(snapshot->z_m, true, 4U, 2U);
    TEXT(" vx="); FIXED(snapshot->vx_mps, true, 3U, 2U);
    TEXT(" vy="); FIXED(snapshot->vy_mps, true, 3U, 2U);
    TEXT(" vz="); FIXED(snapshot->vz_mps, true, 3U, 2U); TEXT("\r\n");
    TEXT("LIGHT: "); FIXED(snapshot->light_percent, false, 3U, 1U);
    TEXT(" %\r\nENV: temp="); FIXED(snapshot->temperature_c, true, 3U, 1U);
    TEXT(" C hum="); FIXED(snapshot->humidity_percent, false, 3U, 1U);
    TEXT(" %\r\nSTATE: "); TEXT(state_text(snapshot->state));
    TEXT("\r\nFLAGS: ");
    if (rc == 0) {
        rc = append_hex16(output, &used, capacity, snapshot->flags);
    }
    TEXT("\r\nDIAGNOSTICS: imu=");
    TEXT((snapshot->flags & TELEMETRY_FLAG_IMU_INVALID) ? "INVALID" : "OK");
    TEXT(" gps=");
    TEXT((snapshot->flags & TELEMETRY_FLAG_GPS_INVALID) ? "NO_FIX_OR_INVALID" : "OK");
    TEXT(" ldr=");
    if (snapshot->ldr_hardware_fault) {
        TEXT("FAULT");
    } else {
        TEXT((snapshot->flags & TELEMETRY_FLAG_LDR_INVALID) ? "INVALID" : "OK");
    }
    TEXT(" si7021=");
    TEXT((snapshot->flags & TELEMETRY_FLAG_SI7021_INVALID) ? "INVALID" : "OK");
    TEXT(" utc=");
    TEXT(snapshot->gps_utc_valid ? "VALID" : "INVALID");
    TEXT(" imu_cal=");
    TEXT((snapshot->flags & TELEMETRY_FLAG_CALIBRATION_PENDING) ? "PENDING" : "DONE");
    TEXT("\r\n");

    if (rc == 0 && include_statistics && snapshot->statistics.valid) {
        rc = append_statistics(output, &used, capacity, &snapshot->statistics);
    }

#undef FIXED
#undef UINT
#undef TEXT
    if (rc != 0 || used >= capacity) {
        if (capacity != 0U) {
            output[0] = '\0';
        }
        return rc != 0 ? rc : -ENOSPC;
    }
    output[used] = '\0';
    return (int)used;
}

int output_format_test_report(const struct telemetry_snapshot *snapshot,
                              char *output, size_t capacity)
{
    return format_report(snapshot, "TEST", false, output, capacity);
}

int output_format_normal_report(const struct telemetry_snapshot *snapshot,
                                bool include_statistics,
                                char *output, size_t capacity)
{
    return format_report(snapshot, "NORMAL", include_statistics,
                         output, capacity);
}
