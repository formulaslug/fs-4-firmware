//
// Created by Jackson Pinsonneault on 10/27/25.
//

#include "debounced_digital_in.h"

int DebouncedDigitalIn::read() const {
    return _current_state;
}

int DebouncedDigitalIn::is_connected() const {
    return _digital_pin.is_connected();
}

void DebouncedDigitalIn::set_valid_read_count(const uint16_t valid_read_count) {
    _valid_read_count = valid_read_count;
}

void DebouncedDigitalIn::add_sample() {
    if (_digital_pin.read() == _current_state) {
        _changed_state_time = 0;
        return;
    }

    _changed_state_time++;

    if (_changed_state_time >= _valid_read_count) {
        _current_state = !_current_state;
        _changed_state_time = 0;
    }
}