//
// Created by Jackson Pinsonneault on 10/27/25.
//

#include "debounce_to_digital_in.h"

int DebounceToDigitalIn::read() const {
    return _current_state;
}

int DebounceToDigitalIn::is_connected() const {
    return _digital_pin.is_connected();
}

void DebounceToDigitalIn::set_valid_read_count(const uint16_t valid_read_count) {
    _valid_read_count = valid_read_count;
}

void DebounceToDigitalIn::add_sample() {
    if (_digital_pin.read() == _current_state) {
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