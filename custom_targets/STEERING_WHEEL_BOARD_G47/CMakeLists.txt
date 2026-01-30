# Copyright (c) 2025 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-steering-wheel-board-g47 INTERFACE)

target_sources(mbed-steering-wheel-board-g47
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-steering-wheel-board-g47
    INTERFACE
        .
)

target_link_libraries(mbed-steering-wheel-board-g47 INTERFACE mbed-stm32g473xe)
