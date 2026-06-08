# set(MBED_UPLOAD_ENABLED true)
# set(MBED_RESET_BAUDRATE 115200)


# Upload Method
if(CMAKE_HOST_WIN32)
    # OpenOCD is too painful to install on windows so we just use pyocd instead,
    # which is slower but still works fine.
    set(UPLOAD_METHOD_DEFAULT PYOCD) 
else()
    set(UPLOAD_METHOD_DEFAULT OPENOCD) 
endif()

set(OPENOCD_UPLOAD_ENABLED true)
set(PYOCD_UPLOAD_ENABLED true)
set(PYOCD_CLOCK_SPEED 10M)

# STLINK Upload Method, here in case OPENOCD and PYOCD doesn't work
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
    MBED_TARGET STREQUAL "CHARGER_BOARD" OR
    MBED_TARGET STREQUAL "STEERING_WHEEL_BOARD" OR
    MBED_TARGET STREQUAL "STM32_TEST_BOARD_V1")

    set(OPENOCD_CHIP_CONFIG_COMMANDS
        -f interface/stlink.cfg
        -c "transport select hla_swd"
        -f target/stm32g4x.cfg
        )
    set(PYOCD_TARGET_NAME STM32G441KBT6)
endif()

#stm32f4x boards
if (MBED_TARGET STREQUAL "TRACTIVE_BATTERY_BOARD" OR
    MBED_TARGET STREQUAL "VEHICLE_CONTROL_UNIT" OR
    MBED_TARGET STREQUAL "NUCLEO_F446RE")

    set(OPENOCD_CHIP_CONFIG_COMMANDS
        -f interface/stlink.cfg
        -c "transport select hla_swd"
        -f target/stm32f4x.cfg
        )
    set(PYOCD_TARGET_NAME STM32F446RET6)
endif()

#stm32l43 boards
if (MBED_TARGET STREQUAL "NUCLEO_L432KC")

    set(OPENOCD_CHIP_CONFIG_COMMANDS
        -f interface/stlink.cfg
        -c "transport select hla_swd"
        -f target/stm32l4x.cfg
        )
    set(PYOCD_TARGET_NAME STM32L432KC)
endif()
