#include "d6t-1a.h"

uint8_t D6T1A::calc_crc(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        uint8_t temp = data;
        data <<= 1;
        if (temp & 0x80) {
            data ^= 0x07;
        }
    }
    return data;
}

// Convert two bytes into signed 16-bit
int16_t D6T1A::le_s16(const uint8_t* buf, int n) {
    uint16_t v = (uint16_t)buf[n];
    v |= ((uint16_t)buf[n + 1]) << 8;
    return (int16_t)v;
}

// PEC check
bool D6T1A::pec_ok(const uint8_t* buf, int payload_len) {
    uint8_t crc = calc_crc((ADDR7 << 1) | 1);
    for (int i = 0; i < payload_len; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    return (crc == buf[payload_len]);
}

bool D6T1A::setup() {
    return true;
}

bool D6T1A::read() {
    std::memset(_rbuf, 0, sizeof(_rbuf));
    const int addr8 = (ADDR7 << 1);

    char cmd = (char)CMD;
    if (_i2c.write(addr8, &cmd, 1, true) != 0) {
        return false;
    }
    if (_i2c.read(addr8, (char*)_rbuf, N_READ, false) != 0) {
        return false;
    }

    if (!pec_ok(_rbuf, N_READ - 1)) {
        return false;
    }

    _ptat_c = (double)le_s16(_rbuf, 0) / 10.0;
    int16_t raw_pix = le_s16(_rbuf, 2);
    _pix_c[0] = (double)raw_pix / 10.0;

    return true;
}
