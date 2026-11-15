/*
 * Deterministic fixed-width SIMU frame serializer (Section 2.4).
 * Uses custom integer/fixed-point ASCII helpers instead of printf("%f") to
 * guarantee exact field widths, sign characters, NaN/Inf safety, and bounded
 * stack usage on Cortex-M4 without C-library float formatting overhead.
 */

#include "simu_protocol.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#define FLAG_GPS_INVALID TELEMETRY_FLAG_GPS_INVALID

/* Return 10^decimals for fixed-point scaling. */
static uint32_t decimal_scale(unsigned int decimals)
{
    uint32_t scale = 1U;

    for (unsigned int i = 0U; i < decimals; ++i) {
        scale *= 10U;
    }
    return scale;
}

/* Append raw bytes into output buffer with strict bounds checking. */
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

/* Format zero-padded unsigned integer of exact width (e.g. SEQ, TMS, GF, SAT). */
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

/* Format 16-bit status bitmask as 4 uppercase hex digits (FLAGS). */
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

/*
 * Format signed or unsigned fixed-point number with mandatory '+'/'-' sign,
 * zero-padded integer digits, '.' separator, and rounded fractional digits.
 * Out-of-range values saturate to the maximum representable fixed-width value;
 * non-finite values (NaN/Inf) fall back safely to zero.
 */
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
        index += decimal_digits;
    }

    return append_bytes(output, used, capacity, field, width);
}

/* Pad tracker state to the mandatory 8-character SIMU field width (Table 4). */
static const char *state_text(enum tracker_state state)
{
    switch (state) {
    case TRACKER_STATE_IDLE:
        return "IDLE____";
    case TRACKER_STATE_FLYING:
        return "FLYING__";
    case TRACKER_STATE_PROBLEM:
        return "PROBLEM_";
    default:
        return "PROBLEM_";
    }
}

/* 8-bit XOR checksum of all ASCII characters strictly between '$' and '*'. */
uint8_t simu_checksum(const char *begin, size_t length)
{
    uint8_t checksum = 0U;

    for (size_t i = 0U; i < length; ++i) {
        checksum ^= (uint8_t)begin[i];
    }
    return checksum;
}

/*
 * Assemble one complete 221-byte SIMU frame:
 * $SIMU1,SEQ,TMS,GF,SAT,LAT,LON,ALT,X,Y,Z,VX,VY,VZ,ROLL,PITCH,YAW,
 * AX,AY,AZ,GX,GY,GZ,LIGHT,TEMP,HUM,STATE,FLAGS*CS\r\n
 */
int simu_format_frame(const struct telemetry_snapshot *snapshot,
                      uint16_t sequence,
                      char output[SIMU_FRAME_BUFFER_SIZE])
{
    size_t used = 0U;
    uint16_t flags;
    uint32_t timestamp_ms;
    uint32_t satellites;
    bool gps_valid;
    int rc;

    if (snapshot == NULL || output == NULL) {
        return -EINVAL;
    }

    /* Reserved bits 8..15 must remain zero; enforce GF=0, SAT=00 on no-fix. */
    flags = snapshot->flags & 0x00FFU;
    gps_valid = snapshot->gps_fix &&
                (snapshot->flags & FLAG_GPS_INVALID) == 0U;
    if (!gps_valid) {
        flags |= FLAG_GPS_INVALID;
    }
    timestamp_ms = snapshot->timestamp_ms % 10000000000ULL;
    satellites = gps_valid ? snapshot->satellites : 0U;
    if (satellites > 99U) {
        satellites = 99U;
    }

    /* Header, 5-digit sequence, 10-digit uptime ms, GPS fix & satellite count. */
    rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, "$SIMU1,");
    if (rc == 0) rc = append_uint(output, &used, SIMU_FRAME_BUFFER_SIZE,
                                  sequence, 5U);
    if (rc == 0) rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, ",");
    if (rc == 0) rc = append_uint(output, &used, SIMU_FRAME_BUFFER_SIZE,
                                  timestamp_ms, 10U);
    if (rc == 0) rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, ",");
    if (rc == 0) rc = append_uint(output, &used, SIMU_FRAME_BUFFER_SIZE,
                                  gps_valid ? 1U : 0U, 1U);
    if (rc == 0) rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, ",");
    if (rc == 0) rc = append_uint(output, &used, SIMU_FRAME_BUFFER_SIZE,
                                  satellites, 2U);

#define APPEND_SIGNED(value, integer_digits, decimal_digits)                \
    do {                                                                     \
        if (rc == 0) {                                                       \
            rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, ",");\
        }                                                                    \
        if (rc == 0) {                                                       \
            rc = append_fixed(output, &used, SIMU_FRAME_BUFFER_SIZE,         \
                              (value), true, (integer_digits),              \
                              (decimal_digits));                             \
        }                                                                    \
    } while (0)

    APPEND_SIGNED(snapshot->latitude_deg, 2U, 6U);
    APPEND_SIGNED(snapshot->longitude_deg, 3U, 6U);
    APPEND_SIGNED(snapshot->altitude_m, 5U, 1U);
    APPEND_SIGNED(snapshot->x_m, 4U, 2U);
    APPEND_SIGNED(snapshot->y_m, 4U, 2U);
    APPEND_SIGNED(snapshot->z_m, 4U, 2U);
    APPEND_SIGNED(snapshot->vx_mps, 3U, 2U);
    APPEND_SIGNED(snapshot->vy_mps, 3U, 2U);
    APPEND_SIGNED(snapshot->vz_mps, 3U, 2U);
    APPEND_SIGNED(snapshot->roll_deg, 3U, 2U);
    APPEND_SIGNED(snapshot->pitch_deg, 3U, 2U);
    APPEND_SIGNED(snapshot->yaw_deg, 3U, 2U);
    APPEND_SIGNED(snapshot->ax_mps2, 3U, 2U);
    APPEND_SIGNED(snapshot->ay_mps2, 3U, 2U);
    APPEND_SIGNED(snapshot->az_mps2, 3U, 2U);
    APPEND_SIGNED(snapshot->gx_dps, 4U, 1U);
    APPEND_SIGNED(snapshot->gy_dps, 4U, 1U);
    APPEND_SIGNED(snapshot->gz_dps, 4U, 1U);

#undef APPEND_SIGNED

    if (rc == 0) rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, ",");
    if (rc == 0) rc = append_fixed(output, &used, SIMU_FRAME_BUFFER_SIZE,
                                   snapshot->light_percent, false, 3U, 1U);
    if (rc == 0) rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, ",");
    if (rc == 0) rc = append_fixed(output, &used, SIMU_FRAME_BUFFER_SIZE,
                                   snapshot->temperature_c, true, 3U, 1U);
    if (rc == 0) rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, ",");
    if (rc == 0) rc = append_fixed(output, &used, SIMU_FRAME_BUFFER_SIZE,
                                   snapshot->humidity_percent, false, 3U, 1U);
    if (rc == 0) rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, ",");
    if (rc == 0) rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE,
                                     state_text(snapshot->state));
    if (rc == 0) rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, ",");
    if (rc == 0) rc = append_hex16(output, &used, SIMU_FRAME_BUFFER_SIZE, flags);
    if (rc == 0) {
        uint8_t checksum = simu_checksum(&output[1], used - 1U);
        static const char digits[] = "0123456789ABCDEF";

        rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, "*");
        if (rc == 0) {
            output[used++] = digits[(checksum >> 4) & 0x0FU];
            output[used++] = digits[checksum & 0x0FU];
        }
    }
    if (rc == 0) rc = append_literal(output, &used, SIMU_FRAME_BUFFER_SIZE, "\r\n");
    if (rc != 0) {
        output[0] = '\0';
        return rc;
    }

    output[used] = '\0';
    if (used != SIMU_FRAME_LENGTH) {
        output[0] = '\0';
        return -EPROTO;
    }

    return 0;
}
