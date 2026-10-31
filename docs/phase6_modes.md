# Phase 6 mode and state management

The application starts in `TEST` and follows the mandatory B1 cycle:

```text
TEST -> NORMAL -> SIMU -> TEST
```

When `CONFIG_APP_ENABLE_ADVANCED_MODE=y`, `ADVANCED` is inserted between
`SIMU` and `TEST`. The transition logic is isolated in `mode_manager.c`, so a
future GPIO interrupt only needs to call `app_b1_button_pressed()` with the
button event timestamp.

## Mode schedules

The default schedules follow the project requirements:

| Mode | Period | LED policy | Serial policy |
|---|---:|---|---|
| TEST | 2 s | Blue | Human-readable diagnostic output (Phase 7 formatter) |
| NORMAL | 5 s | Green IDLE, yellow FLYING, red PROBLEM | Complete report; append the 30 s statistics block at the window boundary |
| SIMU | 100 ms | Magenta, independent of tracker state | Fixed-width SIMU frame only (Phase 7 formatter) |
| ADVANCED | 5 s | White placeholder | Reserved extension point |

The timer is restarted whenever a debounced B1 event changes mode. The output
thread remains the only future VCOM writer. It currently records the selected
LED policy and consumes snapshots; the actual mode formatters and checksum are
implemented in Phase 7.

## Hardware boundary

The mode cycle, debounce rules, schedules, LED mapping, and mock-driven tests
are software-complete. The physical B1 GPIO binding, RGB LED polarity/PWM, and
board-level timing evidence remain hardware verification tasks.
