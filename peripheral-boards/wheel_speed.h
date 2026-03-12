#pragma once

#include "mbed.h"

class WheelSpeed {
public:
    /**
     * @brief constructor for wheel speed sensor
     * Part#: ATS668LSM (This has been changed but the output is still a square wave)
     * Datasheet: https://www.allegromicro.com/-/media/files/datasheets/ats668-datasheet.pdf
     * The sensor outputs a square wave which is then read by a GPIO pin
     * Uses a timer and interrupt that triggers on rising edge to computes period -> frequency -> rpm
     * 
     * @param input_pin pin the sensor is connected to on the mcu
     * @param teeth_per_rev number of teeth on the gear being measured
     * @param timeout time in microseconds until speed is 0 with no gear movement
     */
    WheelSpeed(PinName input_pin,
               uint8_t teeth_per_rev,
               uint32_t timeout);

    /**
     * @brief Calculates rpm by taking  (# of teeth passed) / period 
     */
    float update();

private:
    /**
     * @brief Increments teeth_passed on each tooth/rising edge of the square wave
     */
    void onRiseISR();
    
    InterruptIn sensor;
    Timer timer;

    const uint8_t teeth_per_rev;
    const uint32_t timeout;

    uint32_t start_us;
    volatile uint8_t teeth_passed; //Sampled at 100hz, doubt that there would be more than 256 teeth passed in 0.1 seconds

    float rpm;
};
