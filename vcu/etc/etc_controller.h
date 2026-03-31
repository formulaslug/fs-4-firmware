#ifndef ETC_CONTROLLER_H
#define ETC_CONTROLLER_H

#include "AnalogIn.h"
#include "DigitalIn.h"
#include "DigitalOut.h"
#include "PinNames.h"
#include "Timer.h"
#include "mbed.h"
#include <cstdint>

struct ETCState {
    float APPS1_voltage = 0.0f;
    float APPS2_voltage = 0.0f;
    float APPS1_position = 0.0f;
    float APPS2_position = 0.0f;
    float APPS_position_avg = 0.0f;
    float BPPS_voltage = 0.0f;
    float BPPS_position = 0.0f;
    float front_BSE_voltage = 0.0f;
    float rear_BSE_voltage = 0.0f;
    bool RTD_button_active = false;
    bool RTD_state = false;
    bool motor_enabled = false;
    bool APPS_deviation_implaus = false;
    bool APPS_range_implaus = false;
    bool BPPS_range_implaus = false;
    bool brake_and_accel_implaus = false;
    bool TS_active = false;
    bool can_regen = false;
    bool must_use_hydraulic_brakes = false;
};

class ETCController {
public:
    ETCController(PinName APPS1_pin, PinName APPS2_pin, PinName BPPS_pin, PinName front_BSE_pin, PinName rear_BSE_pin, PinName RTD_button_pin, PinName RTD_light_pin, PinName RTD_buzzer_pin, PinName BSPD_fault_pin);

    bool update_state();
    
    static float clamp(float value);

    static bool in_range(float value, float low, float high, float margin);

    void update_implaus();

    void update_RTD(bool GLV_charged);

    void update_regen(float speed);

    void update_implaus_timer(Timer &timer, bool &timer_running, float min_time, bool implaus_state, bool &etc_implaus); 

    void toggle_RTD(bool new_RTD_state);

    ETCState get_ETC_state();
private:
    ETCState ETC_state;

    AnalogIn APPS1_input;
    AnalogIn APPS2_input;
    AnalogIn BPPS_input;
    AnalogIn front_BSE_input;
    AnalogIn rear_BSE_input;

    DigitalIn RTD_button;
    DigitalIn BSPD_fault_input;

    DigitalOut RTD_light;
    DigitalOut RTD_buzzer;

    Timeout RTD_buzzer_timeout;

    static constexpr std::chrono::seconds RTD_buzzer_duration = 2s;

    static constexpr float APPS1_min_voltage = 0.3125f;
    static constexpr float APPS1_max_voltage = 2.8125f;

    static constexpr float APPS2_min_voltage = 0.1875f;
    static constexpr float APPS2_max_voltage = 1.6875f;

    static constexpr float BPPS_min_voltage = 0.3125f;
    static constexpr float BPPS_max_voltage = 2.8125f;

    static constexpr float PS_voltage_margin = 0.05f;

    static constexpr float front_BSE_activation_voltage = 0.5f;
    static constexpr float rear_BSE_activation_voltage = 0.5f;
    static constexpr float BPPS_max_non_regen_braking = 0.9f;
    
    static constexpr float BPPS_brake_engage_percent = 0.1f;
    static constexpr float max_APPS_position_deviation = 0.10f;
    static constexpr float implaus_min_time = 0.100f; // when greater than 0.1s, activate implaus

    Timer APPS_deviation_implaus_timer;
    Timer APPS_range_implaus_timer;
    Timer BPPS_range_implaus_timer;
    Timer brake_and_accel_implaus_timer;
    bool APPS_deviation_implaus_timer_runnning = false;
    bool APPS_range_implaus_timer_running = false;
    bool BPPS_range_implaus_timer_running = false;
    bool brake_and_accel_implaus_timer_running = false;
};

#endif
