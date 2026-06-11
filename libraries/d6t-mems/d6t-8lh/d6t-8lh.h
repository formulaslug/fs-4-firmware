#pragma once
#include "mbed.h"
#include <cstdint>

class D6T8LH {
public:
    static constexpr uint8_t ADDR7 = 0x0A;
    static constexpr uint8_t CMD = 0x4C;
    static constexpr int N_PIXEL = 8;
    static constexpr int N_READ = ((N_PIXEL + 1) * 2 + 1);

    explicit D6T8LH(I2C& i2c): _i2c(i2c) {}

    // Write start sequence
    bool setup();

    // Read temperatures from sensor. Use pixels_c() to access readings
    int read();

    // Internal reference temperature ("Proportional To Absolute Temperature")
    double ptat_c() const { return _ptat_c; }

    // Temperature of pixels in Celcius. Length N_PIXEL
    const double* pixels_c() const { return _pix_c; }

private:
    I2C& _i2c;
    uint8_t _rbuf[N_READ] = {0};
    double _ptat_c = 0.0;
    double _pix_c[N_PIXEL] = {0};

    static uint8_t calc_crc(uint8_t data);
    static int16_t le_s16(const uint8_t* buf, int n);
    static bool pec_ok(const uint8_t* buf, int payload_len);
};
