# Copyright (c) 2025 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-steering-dash-board INTERFACE)

target_sources(mbed-steering-dash-board
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-steering-dash-board
    INTERFACE
        .
)

target_link_libraries(mbed-steering-dash-board INTERFACE mbed-stm32g473xe)
