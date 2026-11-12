cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(drone_telemetry)

target_sources(app PRIVATE
    src/main.c
    src/sensor_contract.c
    src/processing.c
    src/mode_manager.c
    src/simu_protocol.c
    src/output_formatter.c
)

target_include_directories(app PRIVATE include)
