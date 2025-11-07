//
// Created by Jackson Pinsonneault on 10/28/25.
//

#ifndef MBED_OS_SMOOTHTOANALOGIN_H
#define MBED_OS_SMOOTHTOANALOGIN_H

#include <cstdint>
#include "mbed.h"

class SmoothToAnalogIn {

public:
    /** Create a smoothed AnalogIn using rolling-average
     *
     * @param analog_pin Reference to an AnalogIn to use for reading
     * @param window_size (optional) Rolling-average window size (defaults to 3)
     * @param force_repopulate_time (optional) Time (in milliseconds) where a repopulation of the rolling-average must occur if the last read was greater than this time (defaults to 100ms)
     *
     * @note If force_repopulate_time has passed since the last read, then a repopulation of the rolling-average window will occur with 1ms delays for between each reread
    */
    explicit SmoothToAnalogIn(AnalogIn &analog_pin, const uint16_t window_size=3, const uint16_t force_repopulate_time=100) :
        _analog_pin(analog_pin),
        _force_repopulate_time(force_repopulate_time),
        _window_size(window_size) {
        _held_values = new float[_window_size];
        _timer.start();
    }

    /** Read the referenced AnalogIn input voltage as a rolling-average value, represented as a float in the range [0.0, 1.0]
     *
     * @returns A floating-point value representing the rolling-average of the last window_size reads, measured as a percentage
     *
     * @note If force_repopulate_time has passed since the last read, then a repopulation of the rolling-average window will occur with 1ms delays for between each reread
    */
    float read();

    /** Read the referenced AnalogIn input voltage as a rolling-average value, represented as an unsigned short in the range [0x0, 0xFFFF]
     *
     * @returns 16-bit unsigned short representing the rolling-average of the last window_size reads, normalized to a 16-bit value
     *
     * @note If force_repopulate_time has passed since the last read, then a repopulation of the rolling-average window will occur with 1ms delays for between each reread
    */
    unsigned short read_u16();

    /** Read the referenced AnalogIn input voltage as a rolling-average in volts. The output depends on the target board's
     * ADC reference voltage (typically equal to supply voltage). The ADC reference voltage
     * sets the maximum voltage the ADC can quantify (ie: ADC output == ADC_MAX_VALUE when Vin == Vref)
     *
     * The target's default ADC reference voltage is determined by the configuration
     * option target.default-adc_vref. The reference voltage for a particular input
     * can be manually specified by either the constructor or `AnalogIn::set_reference_voltage`.
     *
     * @returns A floating-point value representing the rolling-average of the last window_size reads, measured in volts.
     *
     * @note If force_repopulate_time has passed since the last read, then a repopulation of the rolling-average window will occur with 1ms delays for between each reread
    */
    float read_voltage();

    /** Sets the referenced AnalogIn's reference voltage.
     *
     * The AnalogIn's reference voltage is used to scale the output when calling AnalogIn::read_volts
     *
     * @param[in] vref New ADC reference voltage for the referenced AnalogIn.
    */
    void set_reference_voltage(float vref);

    /** Gets the referenced AnalogIn's reference voltage.
     *
     * @returns A floating-point value representing the referenced AnalogIn's reference voltage, measured in volts.
    */
    float get_reference_voltage() const;

private:
    AnalogIn &_analog_pin;              //  Referenced AnalogIn which is used for reads
    uint16_t _force_repopulate_time;    //  If this much time (in ms) passes since the last read, then run populate procedure
    uint16_t _window_size;              //  Rolling-average window size
    float* _held_values = nullptr;      //  Rolling-average previously read values
    float _held_values_summed = 0;      //  Sum of the last rolling-average window values
    bool _prepopulated = false;         //  States whether the rolling-average window has been filled
    Timer _timer;                       //  Detects when a forced repopulate should occur

    /** Updates the current rolling-average window to have sample as its newest value
     *
     * @param sample A floating-point value representing the current input voltage, measured as a percentage
    */
    void _add_sample(float sample);
};

#endif //MBED_OS_SMOOTHTOANALOGIN_H