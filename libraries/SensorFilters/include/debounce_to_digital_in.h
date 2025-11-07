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
     * @param pooling_count (optional) Amount of checks that occur when a rechecking of the referenced DigitalIn happens (defaults to 15)
     * @param force_recheck_time (optional) How much time (in milliseconds) can be between each read before a rechecking of the referenced DigitalIn happens (defaults to 100ms)
     *
     * @note If force_recheck_time has passed since the last read, then a rechecking to see if the referenced DigitalIn is still correct must occur, taking 1ms for every check it makes
    */
    explicit DebounceToDigitalIn(DigitalIn &digital_pin, const uint16_t valid_read_count=5, const uint16_t pooling_count=15, const uint16_t force_recheck_time=100) :
        _digital_pin(digital_pin),
        _valid_read_count(valid_read_count),
        _pooling_count(pooling_count),
        _force_recheck_time(force_recheck_time)
        { _timer.start(); }

    /** Read the debounced referenced DigitalIn input, represented as 0 or 1 (int)
     *
     * @returns An integer representing the debounced state of the referenced DigitalIn input, 0 for logical 0, 1 for logical 1
     *
     * @note If force_recheck_time has passed since the last read, then a rechecking to see if the referenced DigitalIn is still correct must occur, taking 1ms for every check it makes
     * @note If the current read is different from the debounced state of the last read, then a rechecking to see if the referenced DigitalIn is still correct must occur, taking 1ms for every check it makes
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

    /** Sets how many samples are taken when rechecking if the referenced DigitalIn is correct
     *
     * @param pooling_count Amount of checks that occur when a rechecking of the referenced DigitalIn happens
     *
     * @note Each checking procedure takes this many ms
    */
    void set_pooling_count(uint16_t pooling_count);

    /** Sets how long you can go without rechecking if the referenced DigitalIn is correct
     *
     * @param force_recheck_time How much time (in milliseconds) can be between each read before a rechecking of the referenced DigitalIn happens
    */
    void set_force_recheck_time(uint16_t force_recheck_time);

private:
    DigitalIn &_digital_pin;            //  DigitalIn which is used for the reads
    bool _current_state = false;        //  current debounced state
    uint16_t _valid_read_count;         //  amount of consecutive reads of the same value to change current_state
    uint16_t _changed_state_time = 0;   //  how many consecutive reads (that aren't the same as current_state) have occurred
    uint16_t _pooling_count;            //  number of samples taken in checking procedure (keep in mind that each checking procedure takes this many ms)
    uint16_t _force_recheck_time;       //  if this much time (in ms) passes since the last read, then run checking procedure
    Timer _timer;                       //  timer to detect when a forced recheck should occur

    /** Updates the current debounced state of the referenced DigitalIn
     *
     * @param sample A boolean value representing the recently read referenced DigitalIn
    */
    void _add_sample(bool sample);
};

#endif //LIBRARIES_DEBOUNCE_H