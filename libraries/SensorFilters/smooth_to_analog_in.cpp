//
// Created by Jackson Pinsonneault on 10/28/25.
//

#include "smooth_to_analog_in.h"

float SmoothToAnalogIn::read() {
    const auto time_dif = chrono::duration_cast<std::chrono::milliseconds>(_timer.elapsed_time());
    const float raw_value = _analog_pin.read();
    _timer.reset();

    if (!_is_initialized) {
        _smoothed_value = raw_value;
        _is_initialized = true;
    } else {
        const float exponent = -0.48 * _alpha * static_cast<float>(time_dif.count()) / 1000.0;
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

void SmoothToAnalogIn::set_alpha(const float alpha) {
    _alpha = alpha;
}

void SmoothToAnalogIn::set_alpha(const float alpha, const float sinusoidal_frequency) {
    float log_alpha = log10(alpha);
    float log_sin = log10(sinusoidal_frequency);

    if (log_alpha > log_sin * 2.5) {
        _alpha = alpha * (2.3 - 0.75 * log_alpha + 0.7 * log_sin);
    }
    else {
        _alpha = alpha * (2.9 - log_alpha + log_sin);
    }
}