#include "wheel_speed.hpp"

WheelSpeed::WheelSpeed(PinName input_pin, uint8_t teeth_per_rev)
    : sensor(input_pin),
      teeth_per_rev(teeth_per_rev),
      start_us(0),
      teeth_passed(0),
      rpm(0.0f) {
    timer.start();
    sensor.rise(callback(this, &WheelSpeed::onRiseISR));
}

void WheelSpeed::onRiseISR() {
    // Required to read/write to volatile variable in explicit steps here
    uint8_t teeth_passed_read = teeth_passed;
    teeth_passed_read++;
    teeth_passed = teeth_passed_read;
    //temp = true;
}

float WheelSpeed::update() {
    // The current method is good at high speeds but less accurate at low speeds
    // The previous method was counting the time frame between every tooth, this would be accurate at low speeds but potentially noisy at higher speeds
    // We could also try a weighted sliding window if our data is too noisy
    uint32_t now_us = timer.elapsed_time().count();
    if (start_us == 0) {
        start_us = now_us;
        teeth_passed = 0;
        return 0.0;
    }
    // if(temp)
    // {
    //     temp = false;
    //     printf("Rising Edge detected\n");
    // }
    uint32_t delta_us = now_us - start_us;
    start_us = now_us;
    // Is running and should be called every 100hz
    // Protecting only teeth_passed since it's changed in the interrupt
    core_util_critical_section_enter();
    uint8_t local_teeth_passed = teeth_passed;
    teeth_passed = 0;
    core_util_critical_section_exit();
    // printf("Teeth Passed: %d\n", local_teeth_passed);
    // printf("Total Teeth Passed: %d\n", total_teeth);
    if (local_teeth_passed == 0) {
        return 0.0;
    }
    // Calculates the frequency in seconds, then calculate rpm
    //total_teeth += local_teeth_passed;
    float freq_s = (static_cast<float>(local_teeth_passed) / (delta_us)) * 1e6f;
    rpm = (freq_s / static_cast<float>(teeth_per_rev)) * 60.0f;
    return rpm;
}
