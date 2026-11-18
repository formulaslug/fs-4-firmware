#include "strain_gauge.hpp"

StrainGauge::StrainGauge(PinName mosi, PinName miso, PinName sck, PinName irq_pin, PinName cs_pin)
    : gauge(mosi, miso, sck), irq(irq_pin), cs(cs_pin) {
    
    irq.mode(PullUp);
    irq.fall(callback(this, &StrainGauge::dataReady));
}

std::array<uint32_t, 4> StrainGauge::getLastVals() {
    return lastVals;
} 

void StrainGauge::init() {
    printf("Initializing!");
    
    cs = 1;
    gauge.format(8, 0);
    gauge.frequency(3'000'000);

    cs = 0;

    // Ext Voltage Reference, Internal Clock with no output, No Current Source, ADC Standby Mode
    write8(CONFIG0, 0b00100010);

    // No Prescaler, OSR = 256
    write8(CONFIG1, 0b00001100);

    // Current Bias = 1x, x64 Gain
    write8(CONFIG2, 0b10111001);

    // Continuous Conversion Mode, 32 bit output with Channel ID, no checksum
    write8(CONFIG3, 0b11110000);

    // Using ~IRQ Pin with pull up
    write8(IRQ, 0b00000011);

    // SCAN[15:0] selects channels to scan through
    write24(SCAN, 0b000000000000111100000000);

    // Converstion state on device with address 00
    gauge.write(0b00101000);

    cs = 1;
}

void StrainGauge::dataReady() {
    uint32_t data = StrainGauge::read32(ADCDATA);
    uint8_t channel = data >> 28;
    data = (data & 0x1FFFFFF);

    if (channel == 0x8) {
        printf("Data from Left Gauge: 0b%" PRIu32 "\n", data);
        lastVals[0] = data;
    } else if (channel == 0x9) {
        printf("Data from Center Gauge: 0b%" PRIu32 "\n", data);
        lastVals[1] = data;
    } else if (channel == 0xA) {
        printf("Data from Right Gauge: 0b%" PRIu32 "\n", data);
        lastVals[2] = data;
    } else if (channel == 0xB) {
        printf("Data from Radial Gauge: 0b%" PRIu32 "\n", data);
        lastVals[3] = data;
    } else {
        printf("Data from unknown channel (0b%" PRIu8 "): 0b%" PRIu32 "\n", channel, data);
    }
}

void StrainGauge::selectWriteAddress(uint8_t addr) {
    const uint8_t txBuf[1] = {
      (uint8_t)(dev_address 
        | addr << 2
        | 0xA) // Incremental Read Mode (00000011)
    };
    gauge.write(txBuf, sizeof(txBuf), nullptr, 0);
}

void StrainGauge::selectReadAddress(uint8_t addr) {
    const uint8_t txBuf[1] = {
      (uint8_t)(dev_address 
        | addr << 2 
        | 0xB) // Single Write Mode (00000010)
    };
   gauge.write(txBuf, sizeof(txBuf), nullptr, 0);
}

void StrainGauge::write8(uint8_t addr, uint8_t data) {
    cs = 0;
    selectWriteAddress(addr);
    gauge.write(data);
    cs = 1;
}

void StrainGauge::write24(uint8_t addr, uint32_t data) {
    cs = 0;
    selectWriteAddress(addr);
    gauge.write(data | 0x00);
    gauge.write((data >> 8) | 0x00);
    gauge.write((data >> 16) | 0x00);
    cs = 1;
}


uint8_t StrainGauge::read8(uint8_t addr) {
    cs = 0;
    
    selectReadAddress(addr);
    uint8_t d =  gauge.write(0x00);
    
    cs = 1;
    return d;
}

uint16_t StrainGauge::read16(uint8_t addr) {
    cs = 0;

    selectReadAddress(addr);
    uint16_t d =  gauge.write(0x00);
    d |=  gauge.write(0x00) << 8;

    cs = 1;
    return d;
}

uint32_t StrainGauge::read32(uint8_t addr) {
    cs = 0;

    selectReadAddress(addr);
    // uint32_t d =  gauge.write(0x000000);
    uint32_t d =  gauge.write(0x00);
    d |=  gauge.write(0x00) << 8;
    d |=  gauge.write(0x00) << 16;
    d |=  gauge.write(0x00) << 24;

    cs = 1;
    return d;
}