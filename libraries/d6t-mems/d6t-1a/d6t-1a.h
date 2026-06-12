#pragma once
#include "mbed.h"
#include <cstdint>

// Read-transaction style — see d6t-8lh.h for details. Keep identical to the D6T8LH setting
// (the #ifndef guard means a -D build flag or whichever header is included first wins).
//   1 = RepeatStart (manual primary), PEC seed = write-addr + cmd + read-addr.
//   0 = Stop-Start (manual 6.5), PEC seed = read-addr only.
#ifndef D6T_USE_REPEATED_START
#define D6T_USE_REPEATED_START 1
#endif

class D6T1A {
public:
    static constexpr uint8_t ADDR7 = 0x0A;
    static constexpr uint8_t CMD = 0x4C;
    static constexpr int N_PIXEL = 1;
    static constexpr int N_READ = ((N_PIXEL + 1) * 2 + 1);

    explicit D6T1A(I2C& i2c) : _i2c(i2c) {}

    // No-op for d6t-1a
    bool setup();

    // Read temperatures from sensor. Use pixel_c() to access reading
    bool read();

    // Internal reference temperature ("Proportional To Absolute Temperature")
    double ptat_c() const { return _ptat_c; }

    // Temperature of pixel in Celcius
    double pixel_c() const { return _pix_c[0]; }

private:
    I2C& _i2c;
    uint8_t _rbuf[N_READ] = {0};
    double  _ptat_c = 0.0;
    double  _pix_c[N_PIXEL] = {0};

    static uint8_t calc_crc(uint8_t data);
    static int16_t le_s16(const uint8_t* buf, int n);
    static bool pec_ok(const uint8_t* buf, int payload_len);
};

