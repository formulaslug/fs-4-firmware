//
// Created by Jackson Pinsonneault on 10/28/25.
//

#include "smooth_to_analog_in.h"

float SmoothToAnalogIn::read() {
    const auto time_dif = chrono::duration_cast<std::chrono::milliseconds>(_timer.elapsed_time());
    _timer.reset();

    if (time_dif >= static_cast<std::chrono::milliseconds>(_force_reset_time)) {
        _is_initialized = false;
    }

    const float raw_value = _analog_pin.read();
    return SmoothToAnalogIn::_compute_EMA(raw_value);
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

float SmoothToAnalogIn::_compute_EMA(const float raw_value) {
    if (!_is_initialized) {
        _smoothed_value = raw_value;
        _is_initialized = true;
    } else {
        _smoothed_value = (_alpha * raw_value) + ((1.0f - _alpha) * _smoothed_value);
    }

    return _smoothed_value;
}