//
// Created by Jackson Pinsonneault on 10/27/25.
//

#include "debounce_to_digital_in.h"

bool DebounceToDigitalIn::read() {
    bool readOutput = _digital_pin.read();
    const auto time_dif = chrono::duration_cast<std::chrono::milliseconds>(timer.elapsed_time());
    timer.reset();
    if (readOutput != current_state || time_dif >= 100ms) {
        add_sample(readOutput);
        ThisThread::sleep_for(std::chrono::milliseconds(1));
        for (int i = 0; i < _pooling_count - 1; i++) {
            readOutput = _digital_pin.read();
            add_sample(readOutput);
            ThisThread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    return current_state;
}

int DebounceToDigitalIn::is_connected() {
    return _digital_pin.is_connected();
}

void DebounceToDigitalIn::set_valid_read_count(uint16_t valid_read_count) {
    _valid_read_count = valid_read_count;
}

void DebounceToDigitalIn::set_pooling_count(uint16_t pooling_count) {
    _pooling_count = pooling_count;
}

void DebounceToDigitalIn::set_force_recheck_time(uint16_t force_recheck_time) {
    _force_recheck_time = force_recheck_time;
}

void DebounceToDigitalIn::add_sample(const bool sample) {
    if (sample == current_state) {
        changed_state_time = 0;
        return;
    }

    if (changed_state_time >= _valid_read_count) {
        current_state = !current_state;
        changed_state_time = 0;
        return;
    }

    changed_state_time++;
}