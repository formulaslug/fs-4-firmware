# set(MBED_UPLOAD_ENABLED true)
# set(MBED_RESET_BAUDRATE 115200) #?

set(UPLOAD_METHOD_DEFAULT OPENOCD)

# OpenOCD configs

set(OPENOCD_UPLOAD_ENABLED true)

#stm32g4x boards
if (MBED_TARGET STREQUAL "PERIPHERAL_BOARD" OR
    MBED_TARGET STREQUAL "STEERING_DASH_BOARD" OR
    MBED_TARGET STREQUAL "STM32_TEST_BOARD_V1" OR
    MBED_TARGET STREQUAL "TRACTIVE_BATTERY_BOARD")

    set(OPENOCD_CHIP_CONFIG_COMMANDS
        -f interface/stlink.cfg
        -c "transport select hla_swd"
        -f target/stm32g4x.cfg
    )

endif()

#stm32f4x boards
if (MBED_TARGET STREQUAL "STEERING_DASH_BOARD_F4")

    set(OPENOCD_CHIP_CONFIG_COMMANDS
        -f interface/stlink.cfg
        -c "transport select hla_swd"
        -f target/stm32f4x.cfg
    )

endif()