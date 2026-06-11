#include "d6t-8lh.h"

uint8_t D6T8LH::calc_crc(uint8_t data) {
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
int16_t D6T8LH::le_s16(const uint8_t *buf, int n) {
    uint16_t ret = (uint16_t)buf[n];
    ret |= ((uint16_t)buf[n + 1]) << 8;
    return (int16_t)ret;
}

// PEC check
bool D6T8LH::pec_ok(const uint8_t* buf, int payload_len)
{
    uint8_t crc;

    crc = calc_crc((ADDR7 << 1) | 0);      // write address
    crc = calc_crc(CMD ^ crc);             // command (0x4C)
    crc = calc_crc((ADDR7 << 1) | 1 ^ crc);// read address

    for (int i = 0; i < payload_len; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }

    return crc == buf[payload_len];
}

int D6T8LH::setup() {
    // mbed I2C needs 8-bit address
    const int addr8 = (ADDR7 << 1);

    const char w1[4] = {0x02, 0x00, 0x01, 0xEE};
    const char w2[4] = {0x05, 0x90, 0x3A, 0xB8};
    const char w3[4] = {0x03, 0x00, 0x03, 0x8B};
    const char w4[4] = {0x03, 0x00, 0x07, 0x97};
    const char w5[4] = {0x02, 0x00, 0x00, 0xE9};

    if (_i2c.write(addr8, w1, 4) != 0) return false;
    if (_i2c.write(addr8, w2, 4) != 0) return false;
    if (_i2c.write(addr8, w3, 4) != 0) return false;
    if (_i2c.write(addr8, w4, 4) != 0) return false;
    if (_i2c.write(addr8, w5, 4) != 0) return false;
    return true;
}

bool D6T8LH::read() {
    std::memset(_rbuf, 0, sizeof(_rbuf));
    const int addr8 = (ADDR7 << 1);
    // write the command byte, then repeated-start read
    char cmd = (char)CMD;
    if (_i2c.write(addr8, &cmd, 1, true) != 0) {
        return 1;
    }
    //read the full packet into rbuf
    if (_i2c.read(addr8, (char*)_rbuf, N_READ, false) != 0) {
        return 2;
    }
    //PEC check (payload is N_READ-1 bytes)
    if (!pec_ok(_rbuf, N_READ - 1)) {
        return 3;
    }

    _ptat_c = (double)le_s16(_rbuf, 0) / 10.0;

    for (int i = 0; i < N_PIXEL; i++) {
        int16_t raw = le_s16(_rbuf, 2 + 2 * i);
        _pix_c[i] = (double)raw / 5.0;
    }

    return 0;
}
