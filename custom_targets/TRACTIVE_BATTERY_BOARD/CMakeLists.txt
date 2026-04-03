# Copyright (c) 2026 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-tractive-battery-board INTERFACE)

target_sources(mbed-tractive-battery-board
    INTERFACE
        PeripheralPins.c
        system_clock.c
)

target_include_directories(mbed-tractive-battery-board
    INTERFACE
        .
)

target_link_libraries(mbed-tractive-battery-board INTERFACE mbed-stm32f446xe)
