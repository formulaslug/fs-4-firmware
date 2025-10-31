# set(MBED_UPLOAD_ENABLED true)
# set(MBED_RESET_BAUDRATE 115200) #?

set(UPLOAD_METHOD_DEFAULT OPENOCD)

# OpenOCD configs

set(OPENOCD_UPLOAD_ENABLED true)
set(OPENOCD_CHIP_CONFIG_COMMANDS
    -f interface/stlink-dap.cfg
    -c "transport select dapdirect_swd"
    -f target/stm32g4x.cfg
)

