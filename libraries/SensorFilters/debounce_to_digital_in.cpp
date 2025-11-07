//
// Created by Jackson Pinsonneault on 10/27/25.
//

#include "debounce_to_digital_in.h"

int DebounceToDigitalIn::read() {
    bool readOutput = _digital_pin.read();
    const auto time_dif = chrono::duration_cast<std::chrono::milliseconds>(_timer.elapsed_time());
    _timer.reset();

    if (readOutput != _current_state || time_dif >= static_cast<std::chrono::milliseconds>(_force_recheck_time)) {
        _add_sample(readOutput);

        for (int i = 0; i < _pooling_count - 1; i++) {
            ThisThread::sleep_for(1ms);

            readOutput = _digital_pin.read();
            _add_sample(readOutput);
        }

        _changed_state_time = 0;
    }

    return _current_state;
}

int DebounceToDigitalIn::is_connected() const {
    return _digital_pin.is_connected();
}

void DebounceToDigitalIn::set_valid_read_count(const uint16_t valid_read_count) {
    _valid_read_count = valid_read_count;
}

void DebounceToDigitalIn::set_pooling_count(const uint16_t pooling_count) {
    _pooling_count = pooling_count;
}

void DebounceToDigitalIn::set_force_recheck_time(const uint16_t force_recheck_time) {
    _force_recheck_time = force_recheck_time;
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