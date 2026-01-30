# set(MBED_UPLOAD_ENABLED true)
# set(MBED_RESET_BAUDRATE 115200)


# OPENOCD Upload Method
set(UPLOAD_METHOD_DEFAULT OPENOCD)
set(OPENOCD_UPLOAD_ENABLED true)


# STLINK Upload Method, here in case OPENOCD doesn't work
#set(UPLOAD_METHOD_DEFAULT = STLINK)
#set(STLINK_UPLOAD_ENABLED true)
#set(STLINK_ARGS --version) # send commands to stlink, no need to uncomment


# STM32CUBE Upload Method, here in case STLINK doesn't work
#set(UPLOAD_METHOD_DEFAULT = STM32CUBE)
#set(STM32CUBE_UPLOAD_ENABLED true)
#set(STM32CUBE_CONNECT_COMMAND port=SWD)
#set(STM32CubeProg_PATH "C:\\Program Files\\STMicroelectronics\\STM32Cube\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe")

#stm32g4x boards
if (MBED_TARGET STREQUAL "PERIPHERAL_BOARD" OR
    MBED_TARGET STREQUAL "STEERING_DASH_BOARD" OR
    MBED_TARGET STREQUAL "STM32_TEST_BOARD_V1" OR
    MBED_TARGET STREQUAL "TRACTIVE_BATTERY_BOARD")

    set(OPENOCD_CHIP_CONFIG_COMMANDS
        -f interface/stlink.cfg
        -c "transport select hla_swd"
        -f target/stm32g4x.cfg
        )   # comment this out if not using OPENOCD

endif()

#stm32f4x boards
if (MBED_TARGET STREQUAL "STEERING_DASH_BOARD_F4" OR
    MBED_TARGET STREQUAL "TRACTIVE_BATTERY_BOARD_F4" OR
    MBED_TARGET STREQUAL "VEHICLE_CONTROL_BOARD_F4")

    set(OPENOCD_CHIP_CONFIG_COMMANDS
        -f interface/stlink.cfg
        -c "transport select hla_swd"
        -f target/stm32f4x.cfg
        )   # comment this out if not using OPENOCD

endif()