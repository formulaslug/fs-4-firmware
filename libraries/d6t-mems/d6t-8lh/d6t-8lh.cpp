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

//  PEC check (Packet Error Check Code). The seed must match the read transaction style (manual section 6.5):
//  - RepeatStart: the sensor's PEC covers write-addr + command + read-addr + data.
//  - Stop-Start:  the command write is a separate transaction, so the PEC covers only
//                 read-addr + data.
bool D6T8LH::pec_ok(const uint8_t* buf, int payload_len)
{
    uint8_t crc = calc_crc((ADDR7 << 1) | 0);      // write address (0x14)
    crc = calc_crc(CMD ^ crc);                     // command (0x4C)
    crc = calc_crc(((ADDR7 << 1) | 1) ^ crc);      // read address (0x15)

    for (int i = 0; i < payload_len; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }

    return crc == buf[payload_len];
}

bool D6T8LH::setup() {
    // mbed I2C needs 8-bit address
    const int addr8 = (ADDR7 << 1); //in write mode

    // D6T-8L-09 init sequence (Omron A284 manual, section 6.4). Must be done once, at least 20 ms after power is applied. Each command is its own START..STOP transaction.
    ThisThread::sleep_for(20ms);

    const char w1[4] = {0x02, 0x00, 0x01, 0xEE};
    const char w2[4] = {0x05, 0x90, 0x3A, 0xB8};
    const char w3[4] = {0x03, 0x00, 0x03, 0x8B};
    const char w4[4] = {0x03, 0x00, 0x07, 0x97};
    const char w5[4] = {0x02, 0x00, 0x00, 0xE9};

    //Nack, timeout, or another error if this happens
    if (_i2c.write(addr8, w1, 4) != 0) return false;
    if (_i2c.write(addr8, w2, 4) != 0) return false;
    if (_i2c.write(addr8, w3, 4) != 0) return false;
    if (_i2c.write(addr8, w4, 4) != 0) return false;
    if (_i2c.write(addr8, w5, 4) != 0) return false;

    // Manual (Fig. 17) specifies a settle wait after init before the first measurement is valid.
    ThisThread::sleep_for(750ms);
    return true;
}

int D6T8LH::read() {
    std::memset(_rbuf, 0, sizeof(_rbuf));
    const int addr8 = (ADDR7 << 1); //setting R/W bit to 0, in write mode, 1 is read
    char cmd = (char)CMD; //0x4C, command to read

    // Write the command byte. D6T_USE_REPEATED_START selects whether the bus is held for a
    // repeated start (manual primary, Fig. 16) or released with a STOP (manual 6.5 alternative).
    if (_i2c.write(addr8, &cmd, 1, true) != 0) {  // true = no STOP -> repeated start before read
        return 1;
    }
    //read the full packet into rbuf
    if (_i2c.read(addr8, (char*)_rbuf, N_READ, false) != 0) {
        return 2;
    }
    //PEC check (payload is N_READ-1 bytes)
    if (!pec_ok(_rbuf, N_READ - 1)) {
        // Debugging
        // uint8_t crc = calc_crc((ADDR7 << 1) | 0);
        // crc = calc_crc(CMD ^ crc);
        // crc = calc_crc(((ADDR7 << 1) | 1) ^ crc);
        // for (int i = 0; i < N_READ - 1; i++) crc = calc_crc(_rbuf[i] ^ crc);
        // printf("D6T raw:");
        // for (int i = 0; i < N_READ; i++) printf(" %02X", _rbuf[i]);
        // printf(" | calcPEC=%02X recvPEC=%02X\n", crc, _rbuf[N_READ - 1]);
        return 3;
    }

    // Per manual section 6.3 Table 3: PTAT and every pixel are (degrees C * 10), signed 16-bit
    // (e.g. 25.0 C == 250). So both divide by 10.
    _ptat_c = (double)le_s16(_rbuf, 0) / 10.0;
    //Reference temperature data stored in the sensor
    // The PTAT and Pn temperature data represents values equal to
    // temperature values (°C) multiplied by a factor of 10 as signed 16-bit
    // integers
    for (int i = 0; i < N_PIXEL; i++) {
        int16_t raw = le_s16(_rbuf, 2 + 2 * i);
        _pix_c[i] = (double)raw / 10.0;
    }

    return 0;
}