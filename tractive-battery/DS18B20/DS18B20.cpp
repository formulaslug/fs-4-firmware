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

    // Search for next device on the bus.
    if (!bus.search(addr)) {
        printf("No more addresses.\r\n\r\n");
        bus.reset_search();
        ThisThread::sleep_for(250ms);
        return;
    }

    // Print the 8-byte ROM address in hex.
    printf("ROM =");
    for (uint8_t byte : addr) {
        printf(" %02X", byte);
    }

    // Check data of the ROM address using CRC8.
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

    // CMD_CONVERT_T (0x44) begins to measure temp.
    // In the case that parasite power is used, the bus will be kept high during conversion.
    bus.write(CMD_CONVERT_T, parasite_power_mode ? 1 : 0);
}

/**
 * @brief Reads the scratchpad memory and processes the raw temp data.
 * @param type_s_sensor Boolean flag; true for DS18S20, false for DS18B20/1822.
 * @return Temperature value scaled to uint8_t (Note: loses fractional precision).
 */
uint8_t DS18B20::retrieve_conversion(bool type_s_sensor) {
    bus.reset();
    bus.select((uint8_t*)&address);
    bus.write(CMD_READ_SCRATCHPAD); // Refers to 0xBE.

    // Read the 9 bytes of the scratchpad (Temp LSB, Temp MSB, TH, TL, Config Register, Res, Res,
    // Res, CRC).
    uint8_t data[9];
    for (int i = 0; i < 9; i++) {
        data[i] = bus.read();
    }

    // Compare 9th byte against the first 8 bytes.
    if (OneWire::crc8(data, 8) != data[8]) {
        return 0xFF; // Return error code if noisy communication.
    }

    // Combine LSB (byte 0) and MSB (byte 1) into one signed 16-bit integer.
    int16_t raw = (data[1] << 8) | data[0];

    // Adjust number based on sensor model.
    if (type_s_sensor) {
        raw = raw << 3; // Shift to default 9-bit resolution.
        if (data[7] == 0x10) {
            // Use 'count remain' to make temp more accurate (12-bit).
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        // Check 'Config' byte to see how precise the sensor is set to be.
        uint8_t cfg = (data[4] & 0x60); // Read config register bits 5 and 6
        if (cfg == 0x00) {
            raw = raw & ~7; // Clean up 3 bits (9-bit res, 93.75 ms).
        } else if (cfg == 0x20) {
            raw = raw & ~3; // Clean up 2 bits (10-bit res, 187.5 ms).
        } else if (cfg == 0x40) {
            raw = raw & ~1; // Clean up 1 bit (11-bit res, 375 ms).
        }
        // Default is a 12-bit res, 750 ms conversion time.
    }

    // float celsius = (float)raw / 16.0;
    // fahrenheit = celsius * 1.8 + 32.0;
    uint8_t celsius_x2 = static_cast<uint8_t>(raw / 8);
    return celsius_x2;
}
