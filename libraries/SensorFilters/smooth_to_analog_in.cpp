//
// Created by Jackson Pinsonneault on 10/28/25.
//

#include "smooth_to_analog_in.h"

float SmoothToAnalogIn::read() {
    float readOutput = _analog_pin.read();
    const auto time_dif = chrono::duration_cast<std::chrono::milliseconds>(_timer.elapsed_time());
    _timer.reset();

    if (time_dif >= static_cast<std::chrono::milliseconds>(_force_repopulate_time)) {
        _prepopulated = false;
    }

    _add_sample(readOutput);

    return _held_values_summed / static_cast<float>(_window_size);
}

unsigned short SmoothToAnalogIn::read_u16() {
    return static_cast<unsigned short>(0xFFFF * read());
}

float SmoothToAnalogIn::read_voltage() {
    return read() * _analog_pin.get_reference_voltage();
}

void SmoothToAnalogIn::set_reference_voltage(float vref) {
    _analog_pin.set_reference_voltage(vref);
}

float SmoothToAnalogIn::get_reference_voltage() const {
    return _analog_pin.get_reference_voltage();
}


void SmoothToAnalogIn::_add_sample(float sample) {
    if (!_prepopulated) {
        _held_values_summed = sample;
        _held_values[_window_size - 1] = sample;
        float readOutput;

        for (int i = _window_size - 2; i >= 0; i--) {
            ThisThread::sleep_for(1ms);

            readOutput = _analog_pin.read();
            _held_values_summed += readOutput;
            _held_values[i] = readOutput;
        }

        _prepopulated = true;
        return;
    }

    _held_values_summed -= _held_values[_window_size - 1];
    _held_values_summed += sample;

    for (int i = _window_size - 2; i >= 0; i--) {
        _held_values[i+1] = _held_values[i];
    }

    _held_values[0] = sample;
}