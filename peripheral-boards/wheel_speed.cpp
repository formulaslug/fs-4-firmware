#include "wheel_speed.h"

WheelSpeed::WheelSpeed(PinName input_pin,
                       uint8_t teeth_per_rev,
                       uint32_t timeout)
    : sensor(input_pin),
      teeth_per_rev(teeth_per_rev),
      timeout(timeout),
      last_us(0),
      period_us(0),
      valid(false),
      rpm(0.0f)
{
    timer.start();
    sensor.rise(callback(this, &WheelSpeed::onRiseISR));
}

void WheelSpeed::onRiseISR()
{
    uint32_t now_us = timer.elapsed_time().count();

    if (valid) {
        period_us = now_us - last_us;
    }
    last_us = now_us;
    valid = true;
}

void WheelSpeed::update()
{
    uint32_t local_period;
    uint32_t local_last;
    bool local_valid;
    //Declares a critical section to stop interrupts, nicer to have the update call reflect the current state rather than after 
    core_util_critical_section_enter(); 
    local_period = period_us;
    local_last = last_us;
    local_valid = valid;
    core_util_critical_section_exit();
    // This critical section doesn't seem to be needed but it guarantees behavior and happens fast enough to not harm anything.

    // (wheel has been stopped)
    if (!local_valid || local_period == 0) {
        rpm = 0.0f;
        return;
    }

    uint32_t now_us = timer.elapsed_time().count();

    // (wheel has just stopped)
    if ((now_us - local_last) > timeout) {
        core_util_critical_section_enter();
        rpm = 0.0f;
        valid = false;
        return;
        core_util_critical_section_exit();
    }

    //Calculates the frequency in hz from microseconds, then calculate rpm
    float freq_hz = 1e6f / static_cast<float>(local_period);
    rpm = (freq_hz / static_cast<float>(teeth_per_rev)) * 60.0f;
}

float WheelSpeed::getRPM() const
{
    return rpm;
}
