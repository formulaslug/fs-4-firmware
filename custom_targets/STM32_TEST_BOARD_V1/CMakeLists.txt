# Copyright (c) 2025 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-stm32-test-board-v1 INTERFACE)

target_sources(mbed-stm32-test-board-v1
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-stm32-test-board-v1
    INTERFACE
        .
)

target_link_libraries(mbed-stm32-test-board-v1 INTERFACE mbed-stm32g441xb)
