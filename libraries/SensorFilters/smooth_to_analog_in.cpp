//
// Created by Jackson Pinsonneault on 10/28/25.
//

#include "smooth_to_analog_in.h"
#include <numbers>

float SmoothToAnalogIn::read() {
    const unsigned long time_dif = chrono::duration_cast<std::chrono::microseconds>(_timer.elapsed_time()).count();
    const float raw_value = _analog_pin.read();
    _timer.reset();

    if (!_is_initialized) {
        _smoothed_value = raw_value;
        _is_initialized = true;
    } else {
        const float exponent = -1.0 * static_cast<float>(time_dif) / pow(10,6) / _time_constant;
        const float exponential_component = exp(exponent);
        _smoothed_value = (1 - exponential_component) * raw_value + exponential_component * _smoothed_value;
    }

    return _smoothed_value;
}

unsigned short SmoothToAnalogIn::read_u16() {
    return static_cast<unsigned short>(0xFFFF * read());
}

float SmoothToAnalogIn::read_voltage() {
    return read() * _analog_pin.get_reference_voltage();
}

void SmoothToAnalogIn::set_reference_voltage(float vref) const {
    _analog_pin.set_reference_voltage(vref);
}

float SmoothToAnalogIn::get_reference_voltage() const {
    return _analog_pin.get_reference_voltage();
}

void SmoothToAnalogIn::set_time_constant(const float cutoff_frequency) {
    _time_constant = 1.0 / (2.0 * M_PI * cutoff_frequency);
}