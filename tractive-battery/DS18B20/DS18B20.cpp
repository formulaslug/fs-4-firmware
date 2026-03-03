#include "DS18B20.h"

/**
 * @brief Constructor for DS18B20 sensor.
 * @param onewire_bus Reference to shared OneWire bus.
 * @param device_address The 64-bit ROM ID for sensor.
 */
DS18B20::DS18B20(OneWire& onewire_bus, uint64_t device_address)
    : bus(onewire_bus), address(device_address) {}
/**
  *@brief Scans the bus for devices and prints their ROM info to serial.
  * Used to indentify unique addresses of sensors.
  */
void debug_search_for_ds18b20_address(OneWire& bus) {
    uint8_t addr[8];

    // Search for next device on the bus
    if (!bus.search(addr)) {
        printf("No more addresses.\r\n\r\n");
        bus.reset_search();
        ThisThread::sleep_for(250ms);
        return;
    }

    // Print the 8-byte ROM address in hex
    printf("ROM =");
    for (uint8_t byte : addr) {
        printf(" %02X", byte);
    }

    // Check data of the ROM address using CRC8
    if (OneWire::crc8(addr, 7) != addr[7]) {
        printf("  CRC is not valid!\r\n\r\n");
        return;
    }

    // Identify the specific chip from the first byte
    switch (addr[0]) {
        case 0x10:
            printf("  Chip = DS18S20 / DS1820 (type_s = 1)\r\n");
            break;
        case 0x28:
            printf("  Chip = DS18B20 (type_s = 0)\r\n");
            break;
        case 0x22:
            printf("  Chip = DS1822 (type_s = 0)\r\n");
            break;
        default:
            printf("  Device is not a DS18x20 family device.\r\n");
            return;
    }
}

/**
 * @brief Send command for temperature conversion.
 * @param parasite_power_mode Set to true if using 2-wire (parasite) mode.
 */
void DS18B20::start_conversion(bool parasite_power_mode) {
    bus.reset();
    bus.select((uint8_t*)&address);
    
    // CMD_CONVERT_T (0x44) begins to measure temp
    // In the case that parasite power is used, the bus will be kept high during conversion
    bus.write(CMD_CONVERT_T, parasite_power_mode ? 1 : 0);
}
