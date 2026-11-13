#include "mbed.h"
#include <cstring>

#include "pin_config.hpp"
#include "address_mapping.hpp"

EventQueue queue = EventQueue{EVENTS_EVENT_SIZE * 32};
SPI gauge(PIN_MOSI, PIN_MISO, PIN_SCK);
DigitalOut cs(PIN_CS);
DigitalIn irq(PIN_IRQ);

uint8_t dev_address = 0x0;

void init();
void pollADC();

void selectWriteAddress(uint8_t addr);
void selectReadAddress(uint8_t addr);

void write8(uint8_t addr, uint8_t data);
void write24(uint8_t addr, uint32_t data);
uint8_t read8(uint8_t addr);
uint16_t read16(uint8_t addr);
uint32_t read24(uint8_t addr);

int main() {
    init();
    queue.call_every(10ms, &pollADC);
}

void init() {
    printf("Initializing!");
    
    cs = 1;
    gauge.format(8, 0);
    gauge.frequency(30*100000);

    cs = 0;

    // Ext Voltage Reference, Internal Clock with no output, No Current Source, ADC Standby Mode
    write8(CONFIG0, 0b00100010);

    // No Prescaler, OSR = 256
    write8(CONFIG1, 0b00001100);

    // Current Bias = 1x, x64 Gain
    write8(CONFIG2, 0b10111001);

    // Continuous Conversion Mode, 24 bit output, no checksum
    write8(CONFIG3, 0b11000000);

    // Using ~IRQ Pin with no pull up
    write8(IRQ, 0b00000111);

    // SCAN[15:0] selects channels to scan through
    write24(SCAN, 0b000000000000111100000000);

    // Converstion state on device with address 00
    gauge.write(0b00101000);

    cs = 1;
}

void pollADC() {
    if (irq) {
        printf("Getting Data!");
        uint32_t data = read24(ADCDATA);
    }
}

void selectWriteAddress(uint8_t addr) {
    const uint8_t txBuf[1] = {
      (uint8_t)(dev_address 
        || addr << 2 
        || 0xA) // Incremental Read Mode (00000011)
    };
    gauge.write(txBuf, sizeof(txBuf), nullptr, 0);
}

void selectReadAddress(uint8_t addr) {
    const uint8_t txBuf[1] = {
      (uint8_t)(dev_address 
        || addr << 2 
        || 0xB) // Single Write Mode (00000010)
    };
   gauge.write(txBuf, sizeof(txBuf), nullptr, 0);
}

void write8(uint8_t addr, uint8_t data) {
    cs = 0;
    selectWriteAddress(addr);
    gauge.write(data);
    cs = 1;
}

void write24(uint8_t addr, uint32_t data) {
    cs = 0;
    selectWriteAddress(addr);
    gauge.write(data | 0x00);
    gauge.write((data >> 8) | 0x00);
    gauge.write((data >> 16) | 0x00);
    cs = 1;
}


uint8_t read8(uint8_t addr) {
    cs = 0;
    
    selectReadAddress(addr);
    uint8_t d =  gauge.write(0x00);
    
    cs = 1;
    return d;
}

uint16_t read16(uint8_t addr) {
    cs = 0;

    selectReadAddress(addr);
    uint16_t d =  gauge.write(0x00);
    d |=  gauge.write(0x00) << 8;

    cs = 1;
    return d;
}

uint32_t read24(uint8_t addr) {
    cs = 0;

    selectReadAddress(addr);
    // uint32_t d =  gauge.write(0x000000);
    uint32_t d =  gauge.write(0x00);
    d |=  gauge.write(0x00) << 8;
    d |=  gauge.write(0x00) << 16;

    cs = 1;
    return d;
}