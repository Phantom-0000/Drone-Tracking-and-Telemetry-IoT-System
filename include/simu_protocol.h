#ifndef SIMU_PROTOCOL_H
#define SIMU_PROTOCOL_H

/*
 * Mandatory fixed-width ASCII SIMU frame serializer ($SIMU1,...*CS\r\n)
 * for deterministic 10 Hz communication with the Python flight simulator.
 */

#include <stddef.h>
#include <stdint.h>

#include "telemetry_snapshot.h"

/* 221 printable/terminator bytes plus the C string terminator. */
#define SIMU_FRAME_LENGTH 221U
#define SIMU_FRAME_BUFFER_SIZE (SIMU_FRAME_LENGTH + 1U)

/*
 * Format one complete SIMU frame. The sequence is five decimal digits and
 * wraps naturally at 65535. On success, output contains exactly 221 bytes,
 * including the final CRLF, followed by a NUL terminator.
 */
int simu_format_frame(const struct telemetry_snapshot *snapshot,
                      uint16_t sequence,
                      char output[SIMU_FRAME_BUFFER_SIZE]);

/* Compute 8-bit XOR checksum over all ASCII bytes between '$' and '*'. */
uint8_t simu_checksum(const char *begin, size_t length);

#endif /* SIMU_PROTOCOL_H */
