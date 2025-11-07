//
// Created by Jackson Pinsonneault on 10/27/25.
//

#ifndef LIBRARIES_DEBOUNCE_H
#define LIBRARIES_DEBOUNCE_H

#include <cstdint>
#include "mbed.h"

class DebounceToDigitalIn {

public:
    /** Create an auto-debouncing DigitalIn
     *
     * @param digital_pin Reference to a DigitalIn to use for reading
     * @param valid_read_count (optional) Amount of consecutive reads of the same value needed to change what read() returns (defaults to 5)
     * @param force_reset_time (optional) How much time (in milliseconds) can be between each read before a reset of the referenced DigitalIn debouncing happens (defaults to 100ms)
     *
     * @note If your reading interval > force_reset_time, then debouncing will never occur
    */
    explicit DebounceToDigitalIn(DigitalIn &digital_pin, const uint16_t valid_read_count=5, const uint16_t force_reset_time=100) :
        _digital_pin(digital_pin),
        _valid_read_count(valid_read_count),
        _force_reset_time(force_reset_time)
        { _timer.start(); }

    /** Read the debounced referenced DigitalIn input, represented as 0 or 1 (int)
     *
     * @returns An integer representing the debounced state of the referenced DigitalIn input, 0 for logical 0, 1 for logical 1
     *
     * @note If your reading interval > force_reset_time, then debouncing will never occur
    */
    int read();

    /** Return the referenced DigitalIn's output setting, represented as 0 or 1 (int)
     *
     * @returns Non-zero value if referenced pin is connected to uc GPIO, 0 if referenced gpio object was initialized with NC
    */
    int is_connected() const;

    /** Sets how resistant the debounced state is to changing
     *
     * @param valid_read_count Amount of consecutive reads of the same value needed to change what read() returns
    */
    void set_valid_read_count(uint16_t valid_read_count);

    /** Sets how long you can go without resetting the referenced DigitalIn of its debounce checks
     *
     * @param force_reset_time How much time (in milliseconds) can be between each read before a reset of the referenced DigitalIn happens
    */
    void set_force_reset_time(uint16_t force_reset_time);

private:
    DigitalIn &_digital_pin;            //  DigitalIn which is used for the reads
    bool _current_state = false;        //  current debounced state
    uint16_t _valid_read_count;         //  amount of consecutive reads of the same value to change current_state
    uint16_t _changed_state_time = 0;   //  how many consecutive reads (that aren't the same as current_state) have occurred
    uint16_t _force_reset_time;         //  if this much time (in ms) passes since the last read, then the debouncing of the referenced DigitalIn is reset
    Timer _timer;                       //  timer to detect when a forced recheck should occur

    /** Updates the current debounced state of the referenced DigitalIn
     *
     * @param sample A boolean value representing the recently read referenced DigitalIn
    */
    void _add_sample(bool sample);
};

#endif //LIBRARIES_DEBOUNCE_H