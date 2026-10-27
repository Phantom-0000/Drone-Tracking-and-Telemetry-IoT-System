cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(drone_telemetry)

target_sources(app PRIVATE
    src/main.c
    src/compatibility_hooks.c
)

target_include_directories(app PRIVATE include)
