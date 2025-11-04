//
// Created by Jackson Pinsonneault on 10/28/25.
//

#ifndef MBED_OS_SMOOTHTOANALOGIN_H
#define MBED_OS_SMOOTHTOANALOGIN_H
#include <cstdint>

#include "mbed.h"


template <int window_size = 15>
class SmoothToAnalogIn {
    AnalogIn _analog_pin;
    uint16_t arr[window_size];

public:
    explicit SmoothToAnalogIn(const AnalogIn &analog_pin) : _analog_pin(analog_pin) {}
    float read();
    uint16_t read_u16();
    float read_voltage();
    void set_reference_voltage(float vref);
    float get_reference_voltage();
    void set_window_size(uint16_t window_size);

private:
    void add_sample(bool sample);
};

#endif //MBED_OS_SMOOTHTOANALOGIN_H