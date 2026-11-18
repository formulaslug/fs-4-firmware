#pragma once
#include "PinNames.h"
#include "mbed.h"

class StrainGauge {
public:
    /**
     * @brief constructor for strain gauge
     */
     StrainGauge(PinName mosi, PinName miso, PinName sck, PinName irq_pin, PinName cs_pin);

    void init();
    std::array<uint32_t, 4> getLastVals();

private:
    //Define Registers
    #define ADCDATA                0x0
    #define CONFIG0                0x1
    #define CONFIG1                0x2
    #define CONFIG2                0x3
    #define CONFIG3                0x4
    #define IRQ                    0x5
    #define MUX                    0x6
    #define SCAN                   0x7
    #define TIMER                  0x8
    #define OFFSETCAL              0x9
    #define GAINCAL                0xA
    #define LOCK                   0xD
    #define CRCCFG                 0xF

    SPI gauge;
    InterruptIn irq;
    DigitalOut cs;

    uint8_t dev_address = 0x0;

    std::array<uint32_t, 4> lastVals = {0x0, 0x0, 0x0, 0x0};

    void selectWriteAddress(uint8_t addr);
    void selectReadAddress(uint8_t addr);

    void write8(uint8_t addr, uint8_t data);
    void write24(uint8_t addr, uint32_t data);
    uint8_t read8(uint8_t addr);
    uint16_t read16(uint8_t addr);
    uint32_t read32(uint8_t addr);

    void dataReady();
};