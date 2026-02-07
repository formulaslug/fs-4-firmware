# Copyright (c) 2026 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-vehicle-control-unit INTERFACE)

target_sources(mbed-vehicle-control-unit
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-vehicle-control-unit
    INTERFACE
        .
)

target_link_libraries(mbed-vehicle-control-unit INTERFACE mbed-stm32f446xe)
