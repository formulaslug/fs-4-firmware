//
// Created by Jackson Pinsonneault on 3/24/26.
//

#ifndef ETC_CONTROLLER_H
#define ETC_CONTROLLER_H

#include "mbed.h"
#include "../imu/vectornav_imu.h"
#include "debounced_digital_in.h"
#include "filtered_analog_in.h"
#include "traction_control.h"
#include "low_pass"

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
    int16_t unfiltered_motor_torque = 0.0f; // -32767 to 32768
    int16_t MAX_SPEED = 7500;
    uint16_t CHARGE_CURRENT_LIMIT = 150;
    uint16_t DISCHARGE_CURRENT_LIMIT = 600;
    uint8_t mbb_alive = 0;
    bool rtd_button_pressed = false;
    bool ready_to_drive = false;
    bool motor_enabled = false;
    bool implaus_APPS_deviation = false;
    bool implaus_APPS_range = false;
    bool implaus_BPPS_range = false;
    bool implaus_BSE_range = false;
    bool implaus_brake_and_accel = false;
    bool regen_allowed = false;
    // solenoid_open: false means the solenoid will be closed (default state)
    // and lets hydraulic brake pressure pass, but true means the solenoid will
    // open and NOT let hydraulic brake pressure through
    bool solenoid_open = false;
    bool reversing = false; // CURRENTLY ISN'T IMPLEMENTED
    bool brakelight_enabled = false;
    float wheel_rpm_fl = 0.0f;
    float wheel_rpm_fr = 0.0f;
    float wheel_rpm_bl = 0.0f;
    float wheel_rpm_br = 0.0f;
    float tc_torque_reduction_factor = 1.0f;
    //VectornavState vectornav;
    uint8_t drive_mode = 0;
    uint8_t traction_mode = 0;
    uint8_t regen_mode = 0;

};

class ETCController {
public:
    ETCState state;
    // Set during CAN reads in main
    bool battery_precharged = false;
    bool shutdown_closed = false;
    TractionController traction_controller;

    ETCController(PinName APPS1_pin, PinName APPS2_pin, PinName BPPS_pin, PinName front_BSE_pin, PinName rear_BSE_pin, PinName rtd_button_pin, PinName rtd_light_pin, PinName rtd_buzzer_pin, PinName solenoid_pin, PinName brakelight_pin, PinName vectornav_tx, PinName vectornav_rx);

    void update_state();

    void update_regen_state(float speed);

    void set_regen_torque(bool is_regening, bool solenoid_open, int16_t regen_torque);

    void update_mbb_alive();

    void turn_off_rtd();
    void turn_on_rtd();

private:
    AnalogIn unfiltered_APPS1_input;
    FilteredAnalogIn APPS1_input;
    AnalogIn unfiltered_APPS2_input;
    FilteredAnalogIn APPS2_input;
    AnalogIn unfiltered_BPPS_input;
    FilteredAnalogIn BPPS_input;
    AnalogIn unfiltered_front_BSE_input;
    FilteredAnalogIn front_BSE_input;
    AnalogIn unfiltered_rear_BSE_input;
    FilteredAnalogIn rear_BSE_input;

    InterruptIn rtd_button;
    DigitalOut rtd_light;
    DigitalOut rtd_buzzer;
    Timeout rtd_buzzer_timeout;

    DigitalOut solenoid;
    DigitalOut brakelight;

    static constexpr std::chrono::seconds RTD_BUZZER_DURATION = 2s;

    static constexpr float APPS1_MIN_VOLTAGE = 0.370f;
    static constexpr float APPS1_DEADZONE_VOLTAGE = 0.03f + APPS1_MIN_VOLTAGE;
    static constexpr float APPS1_MAX_VOLTAGE = 1.040f;

    static constexpr float APPS2_MIN_VOLTAGE = 0.430f;
    static constexpr float APPS2_DEADZONE_VOLTAGE = 0.03f + APPS1_MIN_VOLTAGE;
    static constexpr float APPS2_MAX_VOLTAGE = 1.067f;

    static constexpr float BPPS_MIN_VOLTAGE = 0.550f;
    static constexpr float BPPS_MAX_VOLTAGE = 1.0f; // 2.8125f;

    static constexpr float FRONT_BSE_MIN_VOLTAGE = 0.3125f;
    static constexpr float FRONT_BSE_MAX_VOLTAGE = 2.8125f;

    static constexpr float REAR_BSE_MIN_VOLTAGE = 0.3125f;
    static constexpr float REAR_BSE_MAX_VOLTAGE = 2.8125f;

    // Using BPPS instead
    // static constexpr float FRONT_BSE_ACTIVATION_VOLTAGE = 0.5f;
    // static constexpr float REAR_BSE_ACTIVATION_VOLTAGE = 0.5f;
    static constexpr float BPPS_MAX_NON_REGEN_BRAKING = 0.9f;

    static constexpr float BPPS_BRAKE_ENGAGE_PERCENT = 0.08f;
    static constexpr float MAX_APPS_POSITION_DEVIATION = 0.10f;

    static constexpr int16_t MAX_TORQUE = 32767;
    static constexpr int16_t MAX_REGEN_TORQUE = 32767;

    static constexpr bool REGEN_FORCE_DISABLE = true;
    static constexpr bool TRACTION_CONTROL_FORCE_DISABLE = true;
    static constexpr bool SOLENOID_FORCE_OPEN = true; // open = brake fluid can flow = default state

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

    VectorNavIMU vn_imu;

    static float clamp(float value);

    static bool in_range(float value, float low, float high);

    void update_implaus();

    void update_implaus_timer(Timer &timer, bool &timer_running, bool implaus_state, bool &etc_implaus);

    void rtd_button_irq();
};

#endif
