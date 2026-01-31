# Copyright (c) 2026 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-steering-wheel-board INTERFACE)

target_sources(mbed-steering-wheel-board
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-steering-wheel-board
    INTERFACE
        .
)

target_link_libraries(mbed-steering-wheel-board INTERFACE mbed-stm32g441xb)
