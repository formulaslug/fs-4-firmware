//
// Created by Jackson Pinsonneault on 10/28/25.
//

#include "smooth_to_analog_in.h"

float SmoothToAnalogIn::read() {
    const unsigned long time_dif = chrono::duration_cast<std::chrono::microseconds>(_timer.elapsed_time()).count();
    const float time_average = _add_time_difference(time_dif);
    const float raw_value = _analog_pin.read();
    _timer.reset();

    if (!_is_initialized) {
        _smoothed_value = raw_value;
        _is_initialized = true;
    } else {
        if (_auto_update_alpha) {
            _sampling_frequency = (1.0 / time_average) * pow(10, 6);
        }

        const float exponent = -0.48 * _sampling_frequency * static_cast<float>(time_dif) / pow(10,6); // / 1000.0 * 0.0002
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

void SmoothToAnalogIn::set_alpha(const float sampling_frequency) {
    _sampling_frequency = sampling_frequency;
}

void SmoothToAnalogIn::set_alpha(const float sampling_frequency, const float sinusoidal_frequency) {
    float log_alpha = log10(sampling_frequency);
    float log_sin = log10(sinusoidal_frequency);

    if (log_alpha > log_sin * 2.5) {
        _sampling_frequency = sampling_frequency * (2.3 - 0.75 * log_alpha + 0.7 * log_sin);
    }
    else {
        _sampling_frequency = sampling_frequency * (2.9 - log_alpha + log_sin);
    }
}

float SmoothToAnalogIn::_add_time_difference(const unsigned long time_difference) {
    if (_is_initialized) {
        _time_differences_summed -= _time_differences[9];
        _time_differences_summed += time_difference;

        uint8_t _time_differences_count = 1;
        for (int i = 8; i >= 0; i--) {
            if (_time_differences[i] > 0) {
                _time_differences_count++;
            }
            _time_differences[i+1] = _time_differences[i];
        }

        _time_differences[0] = time_difference;

        return static_cast<float>(_time_differences_summed) / static_cast<float>(_time_differences_count);
    }
    return 0;
}