//
// Created by Jackson Pinsonneault on 10/27/25.
//

#ifndef LIBRARIES_DEBOUNCE_H
#define LIBRARIES_DEBOUNCE_H
#include <cstdint>

#include "mbed.h"

class DebounceToDigitalIn {
    bool current_state = false;
    uint16_t _valid_read_count;
    uint16_t changed_state_time = 0;
    uint16_t _pooling_count;
    uint16_t _force_recheck_time;
    DigitalIn _digital_pin;
    Timer timer;

public:
    explicit DebounceToDigitalIn(const DigitalIn &digital_pin, const uint16_t valid_read_count=5, const uint16_t pooling_count=15, const uint16_t force_recheck_time=100) : _valid_read_count(valid_read_count), _pooling_count(pooling_count), _force_recheck_time(force_recheck_time), _digital_pin(digital_pin) { timer.start(); }
    bool read();
    int is_connected();
    void set_valid_read_count(uint16_t valid_read_count);
    void set_pooling_count(uint16_t pooling_count);
    void set_force_recheck_time(uint16_t force_recheck_time);

private:
    void add_sample(bool sample);
};


#endif //LIBRARIES_DEBOUNCE_H