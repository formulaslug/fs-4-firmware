//
// Created by Jackson Pinsonneault on 3/24/26.
//

#ifndef ETC_CONTROLLER_H
#define ETC_CONTROLLER_H

#include "mbed.h"
#include "debounced_digital_in.h"
#include "filtered_analog_in.h"
#include <cstdint>

struct ETCState {
    float APPS1_voltage = 0.0f;
    float APPS2_voltage = 0.0f;
    float APPS1_position = 0.0f; // 0 to 1
    float APPS2_position = 0.0f; // 0 to 1
    float APPS_position_avg = 0.0f;
    float BPPS_voltage = 0.0f;
    float BPPS_position = 0.0f; // 0 to 1
    float front_BSE_voltage = 0.0f;
    float rear_BSE_voltage = 0.0f;
    int16_t MAX_SPEED = 7500;
    uint16_t CHARGE_CURRENT_LIMIT = 150;
    uint16_t DISCHARGE_CURRENT_LIMIT = 600;
    uint8_t mbb_alive = 0;
    bool rtd_button_pressed = false;
    bool ready_to_drive = false;
    bool motor_enabled = false;
    bool ts_active = false;
    bool implaus_APPS_deviation = false;
    bool implaus_APPS_range = false;
    bool implaus_BPPS_range = false;
    bool implaus_BSE_range = false;
    bool implaus_brake_and_accel = false;
    bool can_regen = false;
    bool must_use_hydraulic_brakes = false;
    float wheel_speed_fl = 0.0f;
    float wheel_speed_fr = 0.0f;
    float wheel_speed_rl = 0.0f;
    float wheel_speed_rr = 0.0f;
};

class ETCController {
public:
    ETCState state;
    bool GLV_ok = false;
    bool shutdown_closed = false;

    ETCController(PinName APPS1_pin, PinName APPS2_pin, PinName BPPS_pin, PinName front_BSE_pin, PinName rear_BSE_pin, PinName rtd_button_pin, PinName rtd_light_pin, PinName rtd_buzzer_pin);

    bool update_state();

    void update_rtd();

    void update_regen(float speed);
private:
    FilteredAnalogIn APPS1_input;
    FilteredAnalogIn APPS2_input;
    FilteredAnalogIn BPPS_input;
    FilteredAnalogIn front_BSE_input;
    FilteredAnalogIn rear_BSE_input;

    DebouncedDigitalIn rtd_button;

    // AnalogIn APPS1_input;
    // AnalogIn APPS2_input;
    // AnalogIn BPPS_input;
    // AnalogIn front_BSE_input;
    // AnalogIn rear_BSE_input;
    //
    // DigitalIn rtd_button;

    DigitalOut rtd_light;
    DigitalOut rtd_buzzer;

    Timeout rtd_buzzer_timeout;

    static constexpr std::chrono::seconds RTD_BUZZER_DURATION = 2s;

    static constexpr float APPS1_MIN_VOLTAGE = 0.3125f;
    static constexpr float APPS1_MAX_VOLTAGE = 2.8125f;

    static constexpr float APPS2_MIN_VOLTAGE = 0.1875f;
    static constexpr float APPS2_MAX_VOLTAGE = 1.6875f;

    static constexpr float BPPS_MIN_VOLTAGE = 0.3125f;
    static constexpr float BPPS_MAX_VOLTAGE = 2.8125f;

    static constexpr float FRONT_BSE_MIN_VOLTAGE = 0.3125f;
    static constexpr float FRONT_BSE_MAX_VOLTAGE = 2.8125f;

    static constexpr float REAR_BSE_MIN_VOLTAGE = 0.3125f;
    static constexpr float REAR_BSE_MAX_VOLTAGE = 2.8125f;

    static constexpr float FRONT_BSE_ACTIVATION_VOLTAGE = 0.5f;
    static constexpr float REAR_BSE_ACTIVATION_VOLTAGE = 0.5f;
    static constexpr float BPPS_MAX_NON_REGEN_BRAKING = 0.9f;
    
    static constexpr float BPPS_BRAKE_ENGAGE_PERCENT = 0.1f;
    static constexpr float MAX_APPS_POSITION_DEVIATION = 0.10f;

    static constexpr int16_t MAX_TORQUE = 30000;

    bool rtd_button_rise = false; // makes it so that update_rtd only occurs on every rise

    Timer implaus_APPS_deviation_timer;
    Timer implaus_APPS_range_timer;
    Timer implaus_BPPS_range_timer;
    Timer implaus_BSE_range_timer;
    Timer implaus_brake_and_accel_timer;
    bool implaus_APPS_deviation_timer_running = false;
    bool implaus_APPS_range_timer_running = false;
    bool implaus_BPPS_range_timer_running = false;
    bool implaus_BSE_range_timer_running = false;
    bool implaus_brake_and_accel_timer_running = false;

    static float clamp(float value);

    static bool in_range(float value, float low, float high);

    void update_implaus();

    void update_implaus_timer(Timer &timer, bool &timer_running, bool implaus_state, bool &etc_implaus); 

    void toggle_rtd(bool rtd_state);
};

#endif
