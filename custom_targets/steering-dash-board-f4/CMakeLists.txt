# Copyright (c) 2025 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-steering-dash-board-f4 INTERFACE)

target_sources(mbed-steering-dash-board-f4
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-steering-dash-board-f4
    INTERFACE
        .
)

target_link_libraries(mbed-steering-dash-board-f4 INTERFACE mbed-stm32f446xe)
