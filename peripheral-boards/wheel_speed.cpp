#include "wheel_speed.hpp"
#include <cstdint>

WheelSpeed::WheelSpeed(PinName input_pin, uint8_t teeth_per_rev)
    : sensor(input_pin), teeth_per_rev(teeth_per_rev), start_us(0), teeth_passed(0), rpm(0.0f) {
    timer.start();
    sensor.rise(callback(this, &WheelSpeed::onRiseISR));

    time_recent_tooth = 0;
}

void WheelSpeed::onRiseISR() {
    // Required to read/write to volatile variable in explicit steps here
    uint8_t teeth_passed_read = teeth_passed;
    teeth_passed_read++;
    teeth_passed = teeth_passed_read;

    if (do_timer_wheelspeed){
        uint64_t _time_rise = timer.elapsed_time().count();
        time_last_cycle = _time_rise - time_recent_tooth;
        time_recent_tooth = _time_rise;
    }
}

// Gets current RPM
float WheelSpeed::update() {
    // The tooth counting method is good at high speeds but less accurate at low speeds
    // The previous method was counting the time frame between every tooth, this would be accurate
    // at low speeds but potentially noisy at higher speeds We could also try a weighted sliding
    // window if our data is too noisy
    //
    // If the number of counted teeth is small, we return a speed based on the time between the last
    // two teeth

    uint32_t now_us = timer.elapsed_time().count();
    if (start_us == 0) {
        start_us = now_us;
        teeth_passed = 0;
        return 0.0;
    }
    uint32_t delta_us = now_us - start_us;
    start_us = now_us;

    // Is running and should be called every 100hz
    // Protecting only teeth_passed since it's changed in the interrupt
    core_util_critical_section_enter();
    uint8_t local_teeth_passed = teeth_passed;
    uint64_t local_recent_tooth = time_recent_tooth;
    uint64_t local_time_last_cycle = time_last_cycle;
    teeth_passed = 0;
    core_util_critical_section_exit();


    if (now_us - local_recent_tooth > 200000) {
        return 0.0;
    }

    if (local_teeth_passed < 20) {
        do_timer_wheelspeed = true;
    } else {
        do_timer_wheelspeed = false;
    }

    //printf("Teeth Passed: %d\n", local_teeth_passed);
    if (local_teeth_passed < 10){
        //printf("Time last cycle us: %d/n", local_time_last_cycle);

        return (60000000.0 / (local_time_last_cycle *  teeth_per_rev));
    }

    // Calculates the frequency in seconds, then calculate rpm
    float freq_s = (static_cast<float>(local_teeth_passed) / (delta_us)) * 1e6f;
    rpm = (freq_s / static_cast<float>(teeth_per_rev)) * 60.0f;
    return rpm;
}
