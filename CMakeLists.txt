cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nrf_virtual_mouse)

target_sources(app PRIVATE
    src/main.c
    src/imu.c
    src/ble_raw_data.c
    src/imu_mouse.c
    src/ble_central.c
    src/ble_hid.c
)
