#pragma once
#include "mbed.h"
#include <cstdint>

class StrainGauge235SL {
public:
    static constexpr uint16_t MAX_WINDOW = 64;
    explicit StrainGauge235SL(PinName adc_pin, float adc_vref = 3.3f);

    //set moving average window size (1 disables filtering)
    //clamped to [1, MAX_WINDOW]
    void set_filter_window(uint16_t window);

    void tare(uint16_t samples = 300, uint32_t sample_delay_us = 200);

    void set_calibration(float slope_units_per_volt, float intercept_units = 0.0f);
    bool set_calibration_two_point(float v1, float units1, float v2, float units2);

    //read latest vals 
    float read_normalized();    //adc
    float read_voltage();   //adc_vref 
    float read_units(); //calibrated output 

    bool  is_saturated_high(float margin_v = 0.02f) const;  //close to adc_vref
    bool  is_saturated_low(float margin_v = 0.02f) const;   //close to 0V

    float tare_voltage() const { return _tare_v; }
    float last_voltage() const { return _last_v; }
    uint16_t filter_window() const { return _win; }

private:
    AnalogIn _ain;
    float _vref;
    float _slope;      
    float _intercept;  
    float _tare_v;    
    uint16_t _win;  //active window size
    float _buf[MAX_WINDOW]; //fixed storage
    uint16_t _idx;  //next write index
    uint16_t _fill; //how many valid samples currently stored 
    float _sum; //sum of samples in current window
    float _last_v;
    float sample_voltage_once();
    float sample_normalized_once();
    float filter_push(float v);
    void  reset_filter_state();
};