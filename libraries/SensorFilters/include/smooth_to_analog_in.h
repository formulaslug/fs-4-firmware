//
// Created by Jackson Pinsonneault on 10/28/25.
//

#ifndef MBED_OS_SMOOTHTOANALOGIN_H
#define MBED_OS_SMOOTHTOANALOGIN_H

#include <cstdint>
#include "mbed.h"

class SmoothToAnalogIn {

public:
    /** Create a smoothed AnalogIn using EMA (exponential moving average)
     *
     * @param analog_pin Reference to an AnalogIn to use for reading
     * @param alpha (optional) Smoothing factor used for EMA (larger the value, the less impact recent values hold)
     * @param force_reset_time (optional) Time (in milliseconds) where the EMA resets if the last read's difference in time was greater than this (defaults to 100ms)
    */
    explicit SmoothToAnalogIn(AnalogIn &analog_pin, const float alpha=0.7, const uint16_t force_reset_time=100) :
        _analog_pin(analog_pin),
        _force_reset_time(force_reset_time),
        _alpha(alpha)
        { _timer.start(); }

    /** Read the referenced AnalogIn input voltage as an EMA value, represented as a float in the range [0.0, 1.0]
     *
     * @returns A floating-point value representing the EMA read, measured as a percentage
     *
     * @note If force_repopulate_time has passed since the last read, then the EMA is reset
    */
    float read();

    /** Read the referenced AnalogIn input voltage as an EMA value, represented as an unsigned short in the range [0x0, 0xFFFF]
     *
     * @returns 16-bit unsigned short representing the EMA read, normalized to a 16-bit value
     *
     * @note If force_repopulate_time has passed since the last read, then the EMA is reset
    */
    unsigned short read_u16();

    /** Read the referenced AnalogIn input voltage as an EMA in volts. The output depends on the target board's
     * ADC reference voltage (typically equal to supply voltage). The ADC reference voltage
     * sets the maximum voltage the ADC can quantify (ie: ADC output == ADC_MAX_VALUE when Vin == Vref)
     *
     * The target's default ADC reference voltage is determined by the configuration
     * option target.default-adc_vref. The reference voltage for a particular input
     * can be manually specified by either the constructor or `AnalogIn::set_reference_voltage`.
     *
     * @returns A floating-point value representing the EMA read, measured in volts.
     *
     * @note If force_repopulate_time has passed since the last read, then the EMA is reset
    */
    float read_voltage();

    /** Sets the referenced AnalogIn's reference voltage.
     *
     * The AnalogIn's reference voltage is used to scale the output when calling AnalogIn::read_volts
     *
     * @param[in] vref New ADC reference voltage for the referenced AnalogIn.
    */
    void set_reference_voltage(float vref) const;

    /** Gets the referenced AnalogIn's reference voltage.
     *
     * @returns A floating-point value representing the referenced AnalogIn's reference voltage, measured in volts.
    */
    float get_reference_voltage() const;

    /** Sets how much recent reads affect the returned value of read()
     *
     * @param alpha Smoothing factor used for EMA (larger the value, the less impact recent values hold)
     *
     * @note 0 < alpha < 1 for proper EMA smoothing
    */
    void set_alpha(float alpha);

private:
    AnalogIn &_analog_pin;          //  Referenced AnalogIn which is used for reads
    uint16_t _force_reset_time;     //  If this much time (in ms) passes since the last read, then reset the EMA
    float _alpha;                   //  The EMA smoothing factor
    float _smoothed_value = 0;      //  The current value that should be returned by read()
    bool _is_initialized = false;   //  States whether the EMA has started
    Timer _timer;                   //  Detects when a forced reset should occur

    /** Calculates the new _smoothed_value according to the _alpha level and the current value
     *
     * @param raw_value A floating-point value representing the current read as a percentage
    */
    float _compute_EMA(float raw_value);
};

#endif //MBED_OS_SMOOTHTOANALOGIN_H