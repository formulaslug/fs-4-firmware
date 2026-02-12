#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include "mbed.h"

class WheelSpeed {
public:
    /**
     * @brief constructor for wheel speed sensor
     * Part#: ATS668LSM
     * Datasheet: https://www.allegromicro.com/-/media/files/datasheets/ats668-datasheet.pdf
     * The sensor outputs a square wave which is read by a GPIO pin
     * Uses a timer and every rising edge triggers an interrupt that computes period -> frequency -> rpm
     * @param input_pin pin the sensor is connected to on the mcu
     * @param teeth_per_rev number of teeth on the gear being measured
     * @param timeout time in microseconds until speed is 0 with no gear movement
     */
    WheelSpeed(PinName input_pin,
               uint8_t teeth_per_rev,
               std::chrono::microseconds timeout);
    /**
     * @brief Returns the rpm
     */          
    float getRPM() const;
    /**
     * @brief Rpm calculations happen here. This should be called in main loop to update the rpm
     * 
     */
    void update();

private:
    /**
     * @brief Interrupt on each rising edge triggers this function. Calculates the time difference between current and last tooth.
     */
    void onRiseISR();
    
    InterruptIn sensor;
    Timer timer;

    const uint8_t teeth_per_rev;
    const std::chrono::microseconds timeout;

    volatile uint64_t last_us;
    volatile uint64_t period_us;
    volatile bool valid;

    float rpm;
};

#endif //WHEEL_SPEED_H
