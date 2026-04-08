//
// Created by Jackson Pinsonneault on 3/24/26.
//

#include "etc_controller.h"

ETCController::ETCController(PinName APPS1_pin, PinName APPS2_pin, PinName BPPS_pin, PinName front_BSE_pin, PinName rear_BSE_pin, PinName rtd_button_pin, PinName rtd_light_pin, PinName rtd_buzzer_pin, PinName BSPD_fault_pin) :
    APPS1_input(APPS1_pin),
    APPS2_input(APPS2_pin),
    BPPS_input(BPPS_pin),
    front_BSE_input(front_BSE_pin),
    rear_BSE_input(rear_BSE_pin),
    rtd_button(rtd_button_pin),
    rtd_light(rtd_light_pin),
    rtd_buzzer(rtd_buzzer_pin),
    BSPD_fault_input(BSPD_fault_pin)
{
    rtd_buzzer.write(0);
    rtd_light.write(0);
}

float ETCController::clamp(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

bool ETCController::in_range(float value, float low, float high) {
    return (value >= low) && (value <= high);
}

bool ETCController::update_state() {
    state.APPS1_voltage = APPS1_input.read_voltage();
    state.APPS2_voltage = APPS2_input.read_voltage();
    state.BPPS_voltage = BPPS_input.read_voltage();
    state.front_BSE_voltage = front_BSE_input.read_voltage();
    state.rear_BSE_voltage = rear_BSE_input.read_voltage();
    state.rtd_button_pressed = rtd_button.read();

    state.APPS1_position = clamp((state.APPS1_voltage - APPS1_MIN_VOLTAGE) / (APPS1_MAX_VOLTAGE - APPS1_MIN_VOLTAGE));
    state.APPS2_position = clamp((state.APPS2_voltage - APPS2_MIN_VOLTAGE) / (APPS2_MAX_VOLTAGE - APPS2_MIN_VOLTAGE));
    state.BPPS_position = clamp((state.BPPS_voltage - BPPS_MIN_VOLTAGE) / (BPPS_MAX_VOLTAGE - BPPS_MIN_VOLTAGE));

    state.APPS_position_avg = (state.APPS1_position + state.APPS2_position) / 2.0f;

    update_implaus();
}

void ETCController::update_implaus_timer(Timer &timer, bool &timer_running, bool implaus_state, bool &etc_implaus) { 
    if (implaus_state) {
        if (etc_implaus) { return; }

        if (!timer_running) {
            timer.reset();
            timer.start();
            timer_running = true;
        } else {
            float time = std::chrono::duration<float>(timer.elapsed_time()).count();
            if (time > 100) {
                etc_implaus = true;
                state.motor_enabled = false;

                timer.stop();
                timer.reset();
                timer_running = false;
            }
        }
    } else {
        etc_implaus = false;

        if (timer_running) {
            timer.stop();
            timer.reset();
            timer_running = false;
        }
    }
}

void ETCController::update_implaus() {
    bool implaus_APPS_deviation = std::abs(state.APPS1_position - state.APPS2_position) > MAX_APPS_POSITION_DEVIATION;
    bool implaus_APPS_range = !in_range(state.APPS1_voltage, APPS1_MIN_VOLTAGE, APPS1_MAX_VOLTAGE) || !in_range(state.APPS2_voltage, APPS2_MIN_VOLTAGE, APPS2_MAX_VOLTAGE);
    bool implaus_BPPS_range = !in_range(state.BPPS_voltage, BPPS_MIN_VOLTAGE, BPPS_MAX_VOLTAGE);
    bool implaus_BSE_range = !in_range(state.front_BSE_voltage, FRONT_BSE_MIN_VOLTAGE, FRONT_BSE_MAX_VOLTAGE) || !in_range(state.rear_BSE_voltage, REAR_BSE_MIN_VOLTAGE, REAR_BSE_MAX_VOLTAGE);
    bool implaus_brake_and_accel = (state.front_BSE_voltage > FRONT_BSE_ACTIVATION_VOLTAGE || state.rear_BSE_voltage > REAR_BSE_ACTIVATION_VOLTAGE) && state.APPS_position_avg > 0.25f;

    update_implaus_timer(implaus_APPS_deviation_timer, implaus_APPS_deviation_timer_running, implaus_APPS_deviation, state.implaus_APPS_deviation);
    update_implaus_timer(implaus_APPS_range_timer, implaus_APPS_range_timer_running, implaus_APPS_range, state.implaus_APPS_range);
    update_implaus_timer(implaus_BPPS_range_timer, implaus_BPPS_range_timer_running, implaus_BPPS_range, state.implaus_BPPS_range);
    update_implaus_timer(implaus_BSE_range_timer, implaus_BSE_range_timer_running, implaus_BSE_range, state.implaus_BSE_range);
    if (state.implaus_brake_and_accel && state.APPS_position_avg < 0.05f) {
        state.implaus_brake_and_accel = false;
    }
    if (implaus_brake_and_accel) {
        state.implaus_brake_and_accel = true;
        state.motor_enabled = false;
    }

    if (!state.motor_enabled && !state.implaus_APPS_deviation && !state.implaus_APPS_range && !state.implaus_BPPS_range && !state.implaus_brake_and_accel && !state.implaus_BSE_range && state.ready_to_drive) {
        state.motor_enabled = true;
    }
}

void ETCController::update_RTD() {
    bool ts_active = state.GLV_ok && state.shutdown_closed; // send BSPD_fault over CAN seperatiely and read Shutdown from BMS CAN

    if (!state.ready_to_drive && ts_active && (state.BPPS_position > BPPS_BRAKE_ENGAGE_PERCENT) && state.rtd_button_pressed) {
        enable_RTD();
    }
}

void ETCController::update_regen(float speed) {
    state.can_regen = !in_range(speed, 0.0f, 5.0f);
    state.must_use_hydraulic_brakes = state.BPPS_position > BPPS_MAX_NON_REGEN_BRAKING;
}

void ETCController::enable_RTD() {
    state.ready_to_drive = true;

    rtd_light.write(1);
    rtd_buzzer.write(1);
    rtd_buzzer_timeout.attach([this]{ rtd_buzzer.write(0); }, RTD_BUZZER_DURATION);
}
