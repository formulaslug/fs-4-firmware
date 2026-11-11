#include "AnalogIn.h"
#include "mbed.h"
#include <cstring>
#include "pin_config.hpp"

EventQueue queue = EventQueue{EVENTS_EVENT_SIZE * 32};
SPI gauge(PIN_MOSI, PIN_MISO, PIN_SCK);
DigitalOut cs(PIN_CS);
DigitalIn irq(PIN_IRQ);

uint8_t address = 0x0;

void init();
void pullData();

int main() {
    init();
    queue.call_every(10ms, &pullData);
}

void pullData() {
    printf("Getting Data!");
    gauge.write(address << 1);
}

void init() {
    printf("Initializing!");
    
    cs = 1;
    gauge.format(8, 0);
    gauge.frequency(30*100000);

    cs = 0;
    gauge.write(0x18A88); // 00101000 - ADC Conversion Start on Device with address 00 (CMD[7:6])
    cs = 1;
}