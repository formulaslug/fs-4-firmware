//
// Created by Jackson Pinsonneault on 3/24/26.
//

#ifndef ETC_CONTROLLER_H
#define ETC_CONTROLLER_H

#include "../imu/vectornav_imu.h"
#include "debounced_digital_in.h"
#include "filtered_analog_in.h"
#include "low_pass_filter.h"
#include "mbed.h"
#include "traction_control.h"

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
    float front_BSE_pressure = 0.0f;
    float read_BSE_pressure = 0.0f;
    int16_t unfiltered_motor_torque = 0.0f; // -32767 to 32768
    int16_t MAX_SPEED = 7500;
    uint16_t CHARGE_CURRENT_LIMIT = 100;        // amps
    uint16_t MAX_DISCHARGE_CURRENT_LIMIT = 600; // amps
    uint16_t DISCHARGE_CURRENT_LIMIT = 570;     // amps
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
    // solenoid_open: true means the solenoid will be open (default state)
    // and lets hydraulic brake pressure pass, but false means the solenoid will
    // close and NOT let hydraulic brake pressure through
    bool solenoid_open = true;
    bool reversing = false; // CURRENTLY ISN'T IMPLEMENTED
    bool brakelight_enabled = false;
    float wheel_rpm_fl = 0.0f;
    float wheel_rpm_fr = 0.0f;
    float wheel_rpm_bl = 0.0f;
    float wheel_rpm_br = 0.0f;
    float tc_torque_reduction_factor = 1.0f;
    VectornavState vectornav;
    uint8_t drive_mode = 0;
    uint8_t traction_mode = 0;
    uint8_t regen_mode = 0;
    // LowPassFilter<int16_t> motor_torque{40}; // example 40hz frequency for -3db filtering

    uint16_t min_battery_voltage = 420; // centivolts
    uint16_t current_draw = 0;          // amps
};

class ETCController {
public:
    bool battery_precharged = false;
    bool shutdown_closed = false;
    TractionController traction_controller;
    ETCState state;

    ETCController(
        PinName APPS1_pin,
        PinName APPS2_pin,
        PinName BPPS_pin,
        PinName front_BSE_pin,
        PinName rear_BSE_pin,
        PinName rtd_button_pin,
        PinName rtd_light_pin,
        PinName rtd_buzzer_pin,
        PinName solenoid_pin,
        PinName brakelight_pin
    ); // add ref to imu at end

    void update_state();

    void update_regen_state(float speed);

    void set_regen_torque(bool is_regening, bool solenoid_open, int16_t regen_torque);

    void update_mbb_alive();

    void turn_off_rtd();

    void turn_on_rtd();

    float current_limit(float voltage, float current);

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

    static constexpr float APPS1_MIN_VOLTAGE_FAULT = 0.300;
    static constexpr float APPS1_MIN_VOLTAGE_EXPECTED = 0.390;
    static constexpr float APPS1_MAX_VOLTAGE_EXPECTED = 1.080f;
    static constexpr float APPS1_MAX_VOLTAGE_FAULT = 1.5f;
    static constexpr float APPS1_DEADZONE_VOLTAGE =
        0.05 * (APPS1_MAX_VOLTAGE_EXPECTED - APPS1_MIN_VOLTAGE_EXPECTED);

    static constexpr float APPS2_MIN_VOLTAGE_FAULT = 0.300f;
    static constexpr float APPS2_MIN_VOLTAGE_EXPECTED = 0.440f;
    static constexpr float APPS2_MAX_VOLTAGE_EXPECTED = 1.145f;
    static constexpr float APPS2_MAX_VOLTAGE_FAULT = 1.5f;
    static constexpr float APPS2_DEADZONE_VOLTAGE =
        0.05 * (APPS2_MAX_VOLTAGE_EXPECTED - APPS2_MIN_VOLTAGE_EXPECTED);

    static constexpr float BPPS_MIN_VOLTAGE_FAULT = 0.300f;
    static constexpr float BPPS_MIN_VOLTAGE_EXPECTED = 0.650f;
    static constexpr float BPPS_MAX_VOLTAGE_EXPECTED = 0.912f; // 2.8125f;
    static constexpr float BPPS_MAX_VOLTAGE_FAULT = 1.5f;      // 2.8125f;
    static constexpr float BPPS_DEADZONE_VOLTAGE =
        0.05 * (BPPS_MAX_VOLTAGE_EXPECTED - BPPS_MIN_VOLTAGE_EXPECTED);

    static constexpr float FRONT_BSE_MIN_VOLTAGE = 0.3125f;
    static constexpr float FRONT_BSE_MAX_VOLTAGE = 2.8125f;

    static constexpr float REAR_BSE_MIN_VOLTAGE = 0.3125f;
    static constexpr float REAR_BSE_MAX_VOLTAGE = 2.8125f;

    // // Using BPPS instead
    // static constexpr float FRONT_BSE_ACTIVATION_VOLTAGE = 0.5f;
    // static constexpr float REAR_BSE_ACTIVATION_VOLTAGE = 0.5f;
    static constexpr float BPPS_MAX_NON_REGEN_BRAKING = 0.8f;

    static constexpr float BPPS_BRAKE_ENGAGE_PERCENT = 0.09f;
    static constexpr float MAX_APPS_POSITION_DEVIATION = 0.10f;

    static constexpr int16_t MAX_TORQUE = 32767 * 0.65;
    static constexpr int16_t MAX_REGEN_TORQUE = 32767 * 0.65;

    static constexpr bool REGEN_FORCE_DISABLE = false;
    static constexpr bool TRACTION_CONTROL_FORCE_DISABLE = true;
    static constexpr bool SOLENOID_FORCE_OPEN =
        false; // open = brake fluid can flow = default state

    static constexpr std::array<std::pair<int, float>, 45> PAD_ROTOR_MU = {{
        (100,0.32875673),
        (101,0.33799832),
        (102,0.34489343),
        (103,0.35039054),
        (104,0.35495863),
        (105,0.35886424),
        (106,0.36227346),
        (107,0.3652968),
        (108,0.36801155),
        (109,0.37047386),
        (110,0.3727258),
        (115,0.3817549),
        (120,0.38841976),
        (125,0.39367559),
        (130,0.39799324),
        (135,0.40163953),
        (140,0.40478022),
        (145,0.40752518),
        (150,0.40995093),
        (155,0.41211282),
        (160,0.41405209),
        (165,0.41580027),
        (170,0.41738196),
        (185,0.42130533),
        (200,0.4242482),
        (215,0.42641952),
        (230,0.42794902),
        (245,0.42892266),
        (260,0.42940081),
        (275,0.42942841),
        (290,0.42904094),
        (305,0.42826816),
        (320,0.4271366),
        (335,0.42567111),
        (350,0.42389609),
        (365,0.4218362),
        (380,0.41951704),
        (395,0.41696552),
        (410,0.41421021),
        (425,0.41128161),
        (440,0.40821231),
        (455,0.40503715),
        (470,0.40179337),
        (485,0.39852067),
        (500,0.39526131)
    }};

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

    // VectorNavIMU &vn_imu;

    static float clamp(float value);

    static bool in_range(float value, float low, float high);

    void update_implaus();

    void
    update_implaus_timer(Timer& timer, bool& timer_running, bool implaus_state, bool& etc_implaus);

    void rtd_button_irq();

    float accelerator_mapping(float pedal_travel);
};

#endif
