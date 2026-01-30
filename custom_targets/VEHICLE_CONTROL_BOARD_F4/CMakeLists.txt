# Copyright (c) 2026 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-vehicle-control-board-f4 INTERFACE)

target_sources(mbed-vehicle-control-board-f4
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-vehicle-control-board-f4
    INTERFACE
        .
)

target_link_libraries(mbed-vehicle-control-board-f4 INTERFACE mbed-stm32f446xe)
