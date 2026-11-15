#ifndef OUTPUT_FORMATTER_H
#define OUTPUT_FORMATTER_H

/*
 * Human-readable ASCII telemetry report formatters for TEST mode (2 s) and
 * NORMAL mode (5 s, including optional 30-second rolling statistics summary).
 */

#include <stdbool.h>
#include <stddef.h>

#include "telemetry_snapshot.h"

/* Format diagnostic telemetry report for TEST mode; returns byte length or <0. */
int output_format_test_report(const struct telemetry_snapshot *snapshot,
                              char *output, size_t capacity);

/* Format operational report for NORMAL mode, appending 30 s stats when requested. */
int output_format_normal_report(const struct telemetry_snapshot *snapshot,
                                bool include_statistics,
                                char *output, size_t capacity);

#endif /* OUTPUT_FORMATTER_H */
