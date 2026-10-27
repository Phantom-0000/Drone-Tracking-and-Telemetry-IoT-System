# Phase 3 Application Foundation

The Phase 3 skeleton establishes the Zephyr application boundary without pretending that the physical sensor wiring is already known. It provides the build entry point, application Kconfig, a board-independent Device Tree application node, the mandatory thread layout, static synchronization objects, the IMU event queue, output timer, and compatibility hooks.

## Current foundation

- `CMakeLists.txt` defines the Zephyr application and source list.
- `Kconfig` freezes the baseline periods: IMU 25 Hz, GPS 1 s, environment 2 s, and SIMU 100 ms.
- `prj.conf` enables the kernel capabilities required by the foundation.
- `app.overlay` defines the application timing node without inventing board-specific pins or sensor addresses.
- `src/main.c` creates acquisition, processing, output, and mode threads, a static IMU queue, a snapshot mutex, a mode semaphore, and a periodic output timer.
- `src/compatibility_hooks.c` provides the required return values and exact compatibility strings. These functions are not called by the runtime SIMU path.

## Deliberately deferred

Physical pin mapping, I2C addresses, UART routing, ADC channel, RGB LED polarity, B1 GPIO, low-level sensor access, mode transitions, telemetry formatting, and SIMU checksum generation remain later integration work. The skeleton emits no startup or debug text, preserving the future SIMU serial ownership rule.

## Phase 3 exit criteria

The application is ready for a Zephyr toolchain build once a target board and the Phase 4 hardware configuration are selected. The required compatibility symbols exist, the task and synchronization structure is present, and the hardware-specific bindings are isolated in the Device Tree/application configuration boundary.

