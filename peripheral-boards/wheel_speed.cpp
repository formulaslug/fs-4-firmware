#include "wheel_speed.h"

WheelSpeed::WheelSpeed(PinName input_pin,
                       uint8_t teeth_per_rev,
                       std::chrono::microseconds timeout)
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
    uint64_t now_us = timer.elapsed_time().count();

    if (valid) {
        period_us = now_us - last_us;
    }

    last_us = now_us;
    valid = true;
}

void WheelSpeed::update()
{
    //(If wheel has been stopped)
    if (!valid || period_us == 0) {
        rpm = 0.0f;
        return;
    }

    uint32_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                     timer.elapsed_time()).count();;

    // timeout check (wheel stopped)
    if ((now_us - last_us) > timeout.count()) {
        rpm = 0.0f;
        return;
    }

    //Calculate the frequency in hz from microseconds
    float freq_hz = 1e6f / static_cast<float>(period_us);
    //Frequency from hz to rotations per minute
    rpm = (freq_hz / static_cast<float>(teeth_per_rev)) * 60.0f;
}

float WheelSpeed::getRPM() const
{
    return rpm;
}
