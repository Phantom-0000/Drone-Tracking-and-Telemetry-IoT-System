# Phase 7 SIMU protocol

The SIMU mode now has a single frame generator owned by the output thread. It
emits the required fixed-width ASCII frame at the existing 100 ms schedule.

## Frame contract

Each frame is exactly 221 bytes including `\r\n`. Numeric fields are rendered
with their required sign, decimal point, and fixed width. The frame contains
the current snapshot, padded state (`IDLE____`, `FLYING__`, or `PROBLEM_`), a
four-digit uppercase FLAGS mask, and a two-digit uppercase XOR checksum.

The checksum covers every byte after `$` and before `*`. GPS no-fix forces
`GF=0`, `SAT=00`, and the GPS-invalid flag. Existing snapshot fallback values
remain in the position fields. The inertial-only and yaw-reliability flags are
passed through from processing.

`tools/check_simu.py` validates a captured binary stream, including frame
length, CRLF, field widths, state values, flags, checksum, and optional 100 ms
timestamp spacing.

## Hardware boundary

Frame construction, fallback behaviour, checksum, and host validation can be
tested with mock snapshots on the host. Final evidence still requires the
board's ST-LINK VCOM at 115200 8N1, a real 100 ms capture, 100 consecutive
frames, and acceptance by the unmodified flight simulator.
