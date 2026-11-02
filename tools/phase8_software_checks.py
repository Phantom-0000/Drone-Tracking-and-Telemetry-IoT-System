#!/usr/bin/env python3
"""Run hardware-independent Phase 8 checks for the telemetry protocol."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_simu import validate_frame  # noqa: E402


def make_frame(sequence: int, timestamp: int, state: str = "IDLE____",
               gps_fix: bool = True, flags: int = 0,
               satellites: int = 8,
               force_satellites: int | None = None) -> bytes:
    if not gps_fix:
        flags |= 0x0002
        satellites = 0 if force_satellites is None else force_satellites
    fields = [
        f"{sequence % 65536:05d}", f"{timestamp % 10000000000:010d}",
        "1" if gps_fix else "0", f"{satellites:02d}",
        "+40.389462", "-003.627416", "+00628.4",
        "+0012.34", "-0003.20", "+0015.00",
        "+000.50", "-000.10", "+001.20",
        "+002.40", "-001.10", "+045.00",
        "-000.12", "+000.04", "+009.81",
        "+0000.5", "-0001.2", "+0000.1",
        "073.5", "+024.6", "045.2", state, f"{flags:04X}",
    ]
    body = "$SIMU1," + ",".join(fields)
    checksum = 0
    for byte in body[1:].encode("ascii"):
        checksum ^= byte
    return (body + f"*{checksum:02X}\r\n").encode("ascii")


def require_valid(frame: bytes, label: str) -> None:
    errors, _sequence, _timestamp = validate_frame(frame)
    if errors:
        raise AssertionError(f"{label}: {errors}")


def require_invalid(frame: bytes, label: str) -> None:
    errors, _sequence, _timestamp = validate_frame(frame)
    if not errors:
        raise AssertionError(f"{label}: invalid frame was accepted")


def main() -> int:
    for state in ("IDLE____", "FLYING__", "PROBLEM_"):
        require_valid(make_frame(1, 100, state=state), f"state {state}")

    require_valid(make_frame(2, 200, gps_fix=False), "GPS fallback")

    valid = make_frame(3, 300)
    require_invalid(valid[:-1], "short frame")
    require_invalid(valid[:-3] + b"00\r\n", "bad checksum")
    require_invalid(make_frame(4, 400, state="BROKEN__"), "bad state")
    require_invalid(make_frame(5, 500, flags=0x0100), "reserved flag")
    require_invalid(make_frame(6, 600, gps_fix=False, force_satellites=8),
                    "GPS fallback satellite mismatch")

    frames = [make_frame(i, i * 100) for i in range(100)]
    for index, frame in enumerate(frames):
        require_valid(frame, f"continuous frame {index}")
    timestamps = [validate_frame(frame)[2] for frame in frames]
    deltas = [timestamps[i] - timestamps[i - 1]
              for i in range(1, len(timestamps))]
    if deltas != [100] * 99:
        raise AssertionError(f"unexpected timestamp deltas: {deltas}")

    print("phase8 software checks: PASS")
    print("cases=states,gps-fallback,checksum,length,flags,timing")
    print("continuous_frames=100 period_ms=100")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
