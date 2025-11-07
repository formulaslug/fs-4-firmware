//
// Created by Jackson Pinsonneault on 10/27/25.
//

#include "debounce_to_digital_in.h"

int DebounceToDigitalIn::read() {
    const bool read_output = _digital_pin.read();
    const auto time_dif = chrono::duration_cast<std::chrono::milliseconds>(_timer.elapsed_time());
    _timer.reset();

    if (time_dif >= static_cast<std::chrono::milliseconds>(_force_reset_time)) {
        _changed_state_time = 0;
        _current_state = read_output;
        return read_output;
    }

    _add_sample(read_output);

    return _current_state;
}

int DebounceToDigitalIn::is_connected() const {
    return _digital_pin.is_connected();
}

void DebounceToDigitalIn::set_valid_read_count(const uint16_t valid_read_count) {
    _valid_read_count = valid_read_count;
}

void DebounceToDigitalIn::set_force_reset_time(const uint16_t force_reset_time) {
    _force_reset_time = force_reset_time;
}

void DebounceToDigitalIn::_add_sample(const bool sample) {
    if (sample == _current_state) {
        _changed_state_time = 0;
        return;
    }

    if (_changed_state_time >= _valid_read_count) {
        _current_state = !_current_state;
        _changed_state_time = 0;
        return;
    }

    _changed_state_time++;
}