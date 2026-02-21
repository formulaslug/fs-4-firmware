#include "wheel_speed.h"

WheelSpeed::WheelSpeed(PinName input_pin,
                       uint8_t teeth_per_rev,
                       uint32_t timeout)
    : sensor(input_pin),
      teeth_per_rev(teeth_per_rev),
      timeout(timeout),
      start_us(0),
      teeth_passed(0),
      rpm(0.0f)
{
    timer.start();
    sensor.rise(callback(this, &WheelSpeed::onRiseISR));
}

void WheelSpeed::onRiseISR()
{
    teeth_passed++;
}

float WheelSpeed::update()
{
    uint32_t now_us = timer.elapsed_time().count();
    uint32_t period;
    uint8_t local_teeth_passed = teeth_passed;

    if(start_us != 0)
    {
        core_util_critical_section_enter;
        period = local_teeth_passed/(now_us - start_us);
        start_us = now_us;
        teeth_passed = 0; 
        core_util_critical_section_exit;
        //In the time this happens, a tooth could of passed
        //Not sure how to fix this issue of tooth counting
    }
    else
    {
        //Update has not been called yet
        start_us = now_us;
        teeth_passed = 0;
        return 0.0;
    }

    //Calculates the frequency in hz from microseconds, then calculate rpm
    float freq_hz = 1e6f / static_cast<float>(period);
    rpm = (freq_hz / static_cast<float>(teeth_per_rev)) * 60.0f;
}
