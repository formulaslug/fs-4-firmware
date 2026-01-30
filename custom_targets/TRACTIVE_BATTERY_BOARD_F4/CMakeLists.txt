# Copyright (c) 2026 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-tractive-battery-board-f4 INTERFACE)

target_sources(mbed-tractive-battery-board-f4
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-tractive-battery-board-f4
    INTERFACE
        .
)

target_link_libraries(mbed-tractive-battery-board-f4 INTERFACE mbed-stm32f446xe)
