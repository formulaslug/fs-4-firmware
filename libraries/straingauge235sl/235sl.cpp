#include "strain_gauge_235sl.h"
#include <cstring> 

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

StrainGauge235SL::StrainGauge235SL(PinName adc_pin, float adc_vref)
    : _ain(adc_pin),
      _vref(adc_vref),
      _slope(1.0f),
      _intercept(0.0f),
      _tare_v(0.0f),
      _win(16),
      _idx(0),
      _fill(0),
      _sum(0.0f),
      _last_v(0.0f)
{ reset_filter_state(); }

//moving avg window size 
void StrainGauge235SL::set_filter_window(uint16_t window) {
    _win = clamp_u16(window, 1, MAX_WINDOW);
    reset_filter_state();
}

//units1,2 = (voltage - tare_voltage) * slope + intercept
void StrainGauge235SL::set_calibration(float slope_units_per_volt, float intercept_units) {
    _slope = slope_units_per_volt;
    _intercept = intercept_units;
}

//two-point calibration helper (only works if v1 & v2 > tare)
//takes points (v1, units1), (v2, units2) and then computes slope of line
bool StrainGauge235SL::set_calibration_two_point(float v1, float units1, float v2, float units2) {
    float dv = (v2 - v1);
    if (dv == 0.0f) return false;
    float du = (units2 - units1);
    _slope = du / dv;
    _intercept = units1 - (_slope * v1);
    return true;
}

float StrainGauge235SL::sample_normalized_once() {
    float n = _ain.read();
    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;
    return n;
}

float StrainGauge235SL::sample_voltage_once() {
    return sample_normalized_once() * _vref;
}

//push new voltage sample into mving average filter
//details: _buf[] stores recent samples, _idx is where it'll write the next sample
float StrainGauge235SL::filter_push(float v) {
    if (_win <= 1) {
        _last_v = v;
        return v;
    }
    //filling window
    if (_fill < _win) {
        _buf[_idx] = v;
        _sum += v;
        _fill++;
        _idx = (_idx + 1) % _win;
        _last_v = _sum / (float)_fill;
        return _last_v;
    }
    //full window 
    _sum -= _buf[_idx];
    _buf[_idx] = v;
    _sum += v;
    _idx = (_idx + 1) % _win;
    _last_v = _sum / (float)_win;
    return _last_v;
}

float StrainGauge235SL::read_normalized() {
    float n = sample_normalized_once();
    float v = n * _vref;
    float vf = filter_push(v);
    return (vf / _vref);
}

float StrainGauge235SL::read_voltage() {
    float v = sample_voltage_once();
    return filter_push(v);
}

float StrainGauge235SL::read_units() {
    float v = read_voltage();
    float dv = v - _tare_v;
    return (dv * _slope) + _intercept;
}

void StrainGauge235SL::tare(uint16_t samples, uint32_t sample_delay_us) {
    if (samples == 0) samples = 1;
    reset_filter_state();
    float acc = 0.0f;
    for (uint16_t i = 0; i < samples; i++) {
        float v = sample_voltage_once();
        float vf = filter_push(v);
        acc += vf;
        if (sample_delay_us) wait_us(sample_delay_us);
    }
    _tare_v = acc / (float)samples;
}

//diagnostic stuff 
bool StrainGauge235SL::is_saturated_high(float margin_v) const {
    return _last_v >= (_vref - margin_v);
} 
bool StrainGauge235SL::is_saturated_low(float margin_v) const {
    return _last_v <= margin_v;
}//

//clears indices, counters, sum, & buffer 
void StrainGauge235SL::reset_filter_state() {
    _idx = 0;
    _fill = 0;
    _sum = 0.0f;
    std::memset(_buf, 0, sizeof(_buf));
    _last_v = 0.0f;
}