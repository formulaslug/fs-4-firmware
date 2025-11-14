//
// Created by Jackson Pinsonneault on 10/28/25.
//

#ifndef MBED_OS_SMOOTHTOANALOGIN_H
#define MBED_OS_SMOOTHTOANALOGIN_H

#include <cstdint>
#include "mbed.h"

class SmoothToAnalogIn {

public:
    /** Create a smoothed AnalogIn using EWMA (exponential weighted moving average)
     *
     * @param analog_pin Reference to an AnalogIn to use for reading
     *
     * @note With this constructor, alpha (the estimated sampling frequency and smoothing) is automatically calculated based off the average time difference between recent reads
    */
    explicit SmoothToAnalogIn(AnalogIn &analog_pin) :
        _analog_pin(analog_pin),
        _auto_update_alpha(true)
    { _timer.start(); }

    /** Create a smoothed AnalogIn using EWMA (exponential weighted moving average)
     *
     * @param analog_pin Reference to an AnalogIn to use for reading
     * @param sampling_frequency Smoothing factor used for EWMA (larger the value = the more impact time differences hold)
     *
     * @note Alpha is tuned by default to use your SAMPLING frequency as its value (however, the more smoothing you want, the lower the value should be)
    */
    explicit SmoothToAnalogIn(AnalogIn &analog_pin, const float sampling_frequency) :
        _analog_pin(analog_pin),
        _sampling_frequency(sampling_frequency)
    { _timer.start(); }

    /** Create a smoothed AnalogIn using EWMA (exponential weighted moving average) with some sinusoidal as your reads
     *
     * @param analog_pin Reference to an AnalogIn to use for reading
     * @param sampling_frequency Smoothing factor used for EWMA (put your SAMPLING frequency)
     * @param sinusoidal_frequency Approximate frequency of the main sinusoidal you are sampling from
     *
     * @note Alpha is tuned by default to use your SAMPLING frequency as its value
     * @note This constructor should ONLY be used when sampling from a noisy sinusoidal pattern
    */
    explicit SmoothToAnalogIn(AnalogIn &analog_pin, const float sampling_frequency, const float sinusoidal_frequency) :
        _analog_pin(analog_pin) {
        _timer.start();
        set_alpha(sampling_frequency, sinusoidal_frequency);
    }

    /** Read the referenced AnalogIn input voltage as an EWMA value, represented as a float in the range [0.0, 1.0]
     *
     * @returns A floating-point value representing the EWMA read, measured as a percentage
    */
    float read();

    /** Read the referenced AnalogIn input voltage as an EWMA value, represented as an unsigned short in the range [0x0, 0xFFFF]
     *
     * @returns 16-bit unsigned short representing the EWMA read, normalized to a 16-bit value
    */
    unsigned short read_u16();

    /** Read the referenced AnalogIn input voltage as an EWMA in volts. The output depends on the target board's
     * ADC reference voltage (typically equal to supply voltage). The ADC reference voltage
     * sets the maximum voltage the ADC can quantify (ie: ADC output == ADC_MAX_VALUE when Vin == Vref)
     *
     * The target's default ADC reference voltage is determined by the configuration
     * option target.default-adc_vref. The reference voltage for a particular input
     * can be manually specified by either the constructor or `AnalogIn::set_reference_voltage`.
     *
     * @returns A floating-point value representing the EWMA read, measured in volts.
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

    /** Sets how much time differences affect the returned value of read()
     *
     * @param sampling_frequency Smoothing factor used for EWMA (larger the value = the more impact time differences hold)
     *
     * @note Alpha is tuned by default to use your sampling frequency as its value (however, the more smoothing you want, the lower the value should be)
    */
    void set_alpha(float sampling_frequency);

    /** Sets how much time differences affect the returned value of read() with some sinusoidal as your reads
     *
     * @param sampling_frequency Smoothing factor used for EWMA (put your SAMPLING frequency)
     * @param sinusoidal_frequency Approximate frequency of the main sinusoidal you are sampling from
     *
     * @note Alpha is tuned by default to use your SAMPLING frequency as its value
     * @note This version of set_alpha should ONLY be used when sampling from a noisy sinusoidal pattern
    */
    void set_alpha(float sampling_frequency, float sinusoidal_frequency);

private:
    AnalogIn &_analog_pin;                      //  Referenced AnalogIn which is used for reads
    float _sampling_frequency;                               //  The EWMA smoothing factor
    float _smoothed_value = 0;                  //  The current value that should be returned by read()
    bool _is_initialized = false;               //  States whether the EWMA has started
    bool _auto_update_alpha;                    //  Tracks whether alpha needs to be updated
    Timer _timer;                               //  Find the difference in times between reads
    unsigned long _time_differences[10] = {0};        //  Holds the time differences between reads
    unsigned long _time_differences_summed = 0; //  Holds the sum of the time differences between reads

    float _add_time_difference(unsigned long time_difference);
};

#endif //MBED_OS_SMOOTHTOANALOGIN_H