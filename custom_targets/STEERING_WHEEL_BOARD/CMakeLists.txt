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

# We link against the stm32g431xb target instead of the stm32g441xb target,
# since for some reasdon only the 431 target includes the mbed system_clock.c
# clock configuration code. Thankfully, the 441 and 431 are virtually identical
# chips (only with AES encryption disabled on the 431), so this works fine.
target_link_libraries(mbed-steering-wheel-board INTERFACE mbed-stm32g431xb)
