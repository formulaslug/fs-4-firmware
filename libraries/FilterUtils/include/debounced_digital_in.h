//
// Created by Jackson Pinsonneault on 10/27/25.
//

#ifndef LIBRARIES_DEBOUNCE_H
#define LIBRARIES_DEBOUNCE_H

#include <cstdint>
#include "mbed.h"

class DebouncedDigitalIn;
inline void sample_all_digital_pins();

inline Ticker debounce_ticker;
inline bool ticker_started = false;
inline std::vector<DebouncedDigitalIn*> digital_pins;

class DebouncedDigitalIn {

public:
    /** Create an auto-debouncing DigitalIn
     *
     * @param digital_pin Reference to a DigitalIn to use for reading
     * @param valid_read_count (optional) Amount of consecutive reads of the same value needed to change what read() returns (defaults to 5)
    */
    explicit DebouncedDigitalIn(DigitalIn &digital_pin, const uint16_t valid_read_count=5) :
        _digital_pin(digital_pin),
        _valid_read_count(valid_read_count) {
        digital_pins.push_back(this);

        if (!ticker_started) {
            debounce_ticker.attach(&sample_all_digital_pins, 1ms);
            ticker_started = true;
        }
    }

    /** Read the debounced referenced DigitalIn input, represented as 0 or 1 (int)
     *
     * @returns An integer representing the debounced state of the referenced DigitalIn input, 0 for logical 0, 1 for logical 1
    */
    int read() const;

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

    /** Updates the current debounced state of the referenced DigitalIn
    */
    void add_sample();

private:
    DigitalIn &_digital_pin;            //  DigitalIn which is used for the reads
    bool _current_state = false;        //  current debounced state
    uint16_t _valid_read_count;         //  amount of consecutive reads of the same value to change current_state
    uint16_t _changed_state_time = 0;   //  how many consecutive reads (that aren't the same as current_state) have occurred
};

inline void sample_all_digital_pins() {
    for (DebouncedDigitalIn *debounced_digital : digital_pins) {
        debounced_digital->add_sample();
    }
}

#endif //LIBRARIES_DEBOUNCE_H