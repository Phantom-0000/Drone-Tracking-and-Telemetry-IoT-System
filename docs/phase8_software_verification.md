# Phase 8 software verification

This record covers the Phase 8 checks that can run before the board and
sensors are available.

## Completed checks

- Default, no-mock, and ADVANCED Zephyr configurations configure and link for
  `nucleo_wl55jc`.
- ROM and RAM footprint reports are generated successfully.
- Required compatibility symbols remain in the final ELF.
- The SIMU checker accepts valid fixed-width frames and rejects malformed
  length, checksum, state, reserved FLAGS, and GPS fallback cases.
- A synthetic 100-frame stream at 100 ms intervals is accepted.
- The protocol checker and Phase 8 software checks pass with the bundled
  Python runtime.

The software audit on 28 October 2026 produced these build results:

| Configuration | Build result | Flash | RAM |
|---|---|---:|---:|
| Default mock sensors | 165/165 steps | 55,848 B (21.30%) | 17,552 B (26.78%) |
| Real-sensor path (`APP_USE_MOCK_SENSORS=n`) | 165/165 steps | 51,004 B (19.46%) | 17,552 B (26.78%) |
| ADVANCED mode | 165/165 steps | 55,856 B (21.31%) | 17,552 B (26.78%) |

The final default ELF retained `config_mandatory_printf`,
`print_alarm_messages`, `print_academic_integrity_marker`,
`simu_format_frame`, `simu_checksum`, and the processing entry points.
The host checks covered all three padded tracker states, GPS no-fix fallback,
bad length, bad checksum, reserved FLAGS, and 100 frames at 100 ms spacing.

## Hardware-dependent checks still open

- Board flash/boot and actual sensor acquisition.
- B1 debounce, RGB LED polarity/colours, and mode transition evidence.
- ST-LINK VCOM 115200 8N1 capture and measured 100 ms output period.
- At least 100 consecutive frames from the board and acceptance by the
  unmodified flight simulator.
- Physical sensor fault, long-duration stability, reset, and stack checks.

The open items are evidence tasks; the software paths and host checks above do
not require hardware.
