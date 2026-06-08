#include "config.hpp"
#include "mbed.h"

// Read DIP switch which encodes which corner the board is on
Corner readCorner() {
    DigitalIn dip1(PIN_DIP_1, PullUp);
    DigitalIn dip2(PIN_DIP_2, PullUp);
    // dip1 is first bit, dip2 is next bit
    int val = (dip2.read() << 1) | dip1.read();
    switch (val) {
    case 0:
        return Corner::FR;
    case 1:
        return Corner::FL;
    case 2:
        return Corner::BR;
    case 3:
        return Corner::BL;
    default:
        return Corner::FR; // Might want to add something for an invald position
    }
}

// Could probably just make a lookup table for this
CornerConfig getCornerConfig(Corner pos) {
    switch (pos) {
    case Corner::FL:
        return {0x1A5, 0x2A5, false, false};

    case Corner::FR:
        return {0x1A6, 0x2A6, false, false};

    case Corner::BL:
        return {0x1A7, 0x2A7, false, false};

    case Corner::BR:
        return {0x1A8, 0x2A8, false, false};

    default:
        return {0x1A6, 0x2A6, false, false};
        // Defaulting to FR, Not sure what to do for an error
        // FS-3 #error "WHEEL_POSITION must be one of BR/BL/FR/FL!"
    }
}
