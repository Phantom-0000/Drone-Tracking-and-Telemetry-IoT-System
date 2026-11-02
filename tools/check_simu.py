#!/usr/bin/env python3
"""Validate Drone Tracker SIMU frames and report capture timing."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

FRAME_LENGTH = 221
STATE_VALUES = {"IDLE____", "FLYING__", "PROBLEM_"}
FIELD_WIDTHS = [5, 10, 1, 2, 10, 11, 8, 8, 8, 8, 7, 7, 7,
                7, 7, 7, 7, 7, 7, 7, 7, 7, 5, 6, 5, 8, 4]


def _fixed_width(value: str, width: int, label: str) -> str | None:
    if len(value) != width:
        return f"{label} width {len(value)} != {width}"
    return None


def _matches(value: str, pattern: str, label: str) -> str | None:
    import re

    if re.fullmatch(pattern, value) is None:
        return f"{label} has invalid format: {value!r}"
    return None


def validate_frame(raw: bytes) -> tuple[list[str], int | None, int | None]:
    errors: list[str] = []
    sequence: int | None = None
    timestamp: int | None = None

    if len(raw) != FRAME_LENGTH:
        errors.append(f"frame length {len(raw)} != {FRAME_LENGTH}")
    if not raw.endswith(b"\r\n"):
        errors.append("frame does not end with CRLF")
        return errors, sequence, timestamp

    try:
        text = raw.decode("ascii")
    except UnicodeDecodeError:
        errors.append("frame is not printable ASCII")
        return errors, sequence, timestamp

    if not text.startswith("$SIMU1,"):
        errors.append("invalid start marker")
        return errors, sequence, timestamp

    body = text[:-2]
    if body.count("*") != 1:
        errors.append("frame must contain exactly one checksum separator")
        return errors, sequence, timestamp

    before_checksum, encoded_checksum = body.rsplit("*", 1)
    if len(encoded_checksum) != 2:
        errors.append("checksum must contain two characters")
    elif _matches(encoded_checksum, r"[0-9A-F]{2}", "checksum"):
        errors.append("checksum is not uppercase hexadecimal")
    else:
        expected = 0
        for char in before_checksum[1:]:
            expected ^= ord(char)
        if encoded_checksum != f"{expected:02X}":
            errors.append(
                f"checksum {encoded_checksum} != expected {expected:02X}"
            )

    fields = before_checksum.split(",")
    if not fields or fields[0] != "$SIMU1":
        errors.append("invalid protocol marker field")
        return errors, sequence, timestamp
    values = fields[1:]
    if len(values) != len(FIELD_WIDTHS):
        errors.append(f"field count {len(values)} != {len(FIELD_WIDTHS)}")
        return errors, sequence, timestamp

    for index, (value, width) in enumerate(zip(values, FIELD_WIDTHS)):
        error = _fixed_width(value, width, f"field {index + 1}")
        if error:
            errors.append(error)

    checks = [
        (values[0], r"\d{5}", "SEQ"),
        (values[1], r"\d{10}", "TMS"),
        (values[2], r"[01]", "GF"),
        (values[3], r"\d{2}", "SAT"),
        (values[4], r"[+-]\d{2}\.\d{6}", "LAT"),
        (values[5], r"[+-]\d{3}\.\d{6}", "LON"),
        (values[6], r"[+-]\d{5}\.\d", "ALT"),
        (values[7], r"[+-]\d{4}\.\d{2}", "X"),
        (values[8], r"[+-]\d{4}\.\d{2}", "Y"),
        (values[9], r"[+-]\d{4}\.\d{2}", "Z"),
        (values[10], r"[+-]\d{3}\.\d{2}", "VX"),
        (values[11], r"[+-]\d{3}\.\d{2}", "VY"),
        (values[12], r"[+-]\d{3}\.\d{2}", "VZ"),
        (values[13], r"[+-]\d{3}\.\d{2}", "ROLL"),
        (values[14], r"[+-]\d{3}\.\d{2}", "PITCH"),
        (values[15], r"[+-]\d{3}\.\d{2}", "YAW"),
        (values[16], r"[+-]\d{3}\.\d{2}", "AX"),
        (values[17], r"[+-]\d{3}\.\d{2}", "AY"),
        (values[18], r"[+-]\d{3}\.\d{2}", "AZ"),
        (values[19], r"[+-]\d{4}\.\d", "GX"),
        (values[20], r"[+-]\d{4}\.\d", "GY"),
        (values[21], r"[+-]\d{4}\.\d", "GZ"),
        (values[22], r"\d{3}\.\d", "LIGHT"),
        (values[23], r"[+-]\d{3}\.\d", "TEMP"),
        (values[24], r"\d{3}\.\d", "HUM"),
        (values[25], r"[A-Z_]{8}", "STATE"),
        (values[26], r"[0-9A-F]{4}", "FLAGS"),
    ]
    for value, pattern, label in checks:
        error = _matches(value, pattern, label)
        if error:
            errors.append(error)

    if values[25] not in STATE_VALUES:
        errors.append(f"STATE is not allowed: {values[25]!r}")

    try:
        flags = int(values[26], 16)
        gps_fix = values[2] == "1"
        satellites = int(values[3])
        if flags & 0xFF00:
            errors.append("FLAGS uses a reserved bit")
        if not gps_fix and satellites != 0:
            errors.append("SAT must be 00 when GF is 0")
        if not gps_fix and not (flags & 0x0002):
            errors.append("GPS-invalid flag must be set when GF is 0")
        if gps_fix and (flags & 0x0002):
            errors.append("GPS-invalid flag must be clear when GF is 1")
    except ValueError:
        pass

    try:
        sequence = int(values[0])
        timestamp = int(values[1])
    except ValueError:
        pass
    return errors, sequence, timestamp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path,
                        help="binary capture file, or '-' for stdin")
    parser.add_argument("--min-frames", type=int, default=1)
    parser.add_argument("--require-10hz", action="store_true",
                        help="require every timestamp delta to be 100 ms")
    args = parser.parse_args()

    data = sys.stdin.buffer.read() if str(args.capture) == "-" \
        else args.capture.read_bytes()
    frames = data.splitlines(keepends=True)
    if len(frames) < args.min_frames:
        print(f"FAIL: only {len(frames)} frames, need {args.min_frames}")
        return 1

    timestamps: list[int] = []
    failures = 0
    for index, frame in enumerate(frames, start=1):
        errors, _sequence, timestamp = validate_frame(frame)
        if errors:
            failures += 1
            for error in errors:
                print(f"frame {index}: {error}")
        if timestamp is not None:
            timestamps.append(timestamp)

    deltas = [b - a for a, b in zip(timestamps, timestamps[1:])]
    if args.require_10hz and any(delta != 100 for delta in deltas):
        print(f"timing deltas are not all 100 ms: {deltas}")
        failures += 1

    print(f"frames={len(frames)} valid={len(frames) - failures} invalid={failures}")
    if deltas:
        print(f"timestamp_deltas_ms={deltas}")
    if failures:
        return 1
    print("SIMU capture accepted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
