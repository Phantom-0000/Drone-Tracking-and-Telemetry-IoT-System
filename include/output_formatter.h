#ifndef OUTPUT_FORMATTER_H
#define OUTPUT_FORMATTER_H

#include <stdbool.h>
#include <stddef.h>

#include "telemetry_snapshot.h"

int output_format_test_report(const struct telemetry_snapshot *snapshot,
                              char *output, size_t capacity);

int output_format_normal_report(const struct telemetry_snapshot *snapshot,
                                bool include_statistics,
                                char *output, size_t capacity);

#endif /* OUTPUT_FORMATTER_H */
