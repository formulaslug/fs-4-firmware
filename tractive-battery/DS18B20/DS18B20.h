
#pragma once

#include "mbed.h"
#include "OneWire.h"

// For debugging only: search for onewire device addresses on the bus
// (useful for setup process)
void debug_search_for_ds18b20_address(OneWire& bus);

class DS18B20 {
public:
    explicit DS18B20(OneWire& onewire_bus, uint64_t device_address);

    // Begin temperature sense process
    void start_conversion(bool parasite_power_mode = false);

    // Retrieves temperature in Degrees Celsius, multiplied by 2 (e.g., 42 ==> 21C)
    // Returns 0xFF (255) if the CRC check fails.
    uint8_t retrieve_conversion(bool type_s_sensor = false);

private:
    OneWire& bus;
    uint64_t address;

    // 1-Wire Commands
    static constexpr uint8_t CMD_CONVERT_T = 0x44;
    static constexpr uint8_t CMD_READ_SCRATCHPAD = 0xBE;
};
