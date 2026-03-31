#include "etc_controller.h"
#include <cmath>
#include <chrono>
#include "PinNames.h"
#include "mbed_chrono.h"

ETCController::ETCController(PinName APPS1_pin, PinName APPS2_pin, PinName BPPS_pin, PinName front_BSE_pin, PinName rear_BSE_pin, PinName RTD_button_pin, PinName RTD_light_pin, PinName RTD_buzzer_pin, PinName BSPD_fault_pin) :
    APPS1_input(APPS1_pin),
    APPS2_input(APPS2_pin),
    BPPS_input(BPPS_pin),
    front_BSE_input(front_BSE_pin),
    rear_BSE_input(rear_BSE_pin),
    RTD_button(RTD_button_pin),
    RTD_light(RTD_light_pin),
    RTD_buzzer(RTD_buzzer_pin),
    BSPD_fault_input(BSPD_fault_pin) {}

float ETCController::clamp(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

bool ETCController::in_range(float value, float low, float high, float margin) {
    return (value >= (low - margin)) && (value <= (high + margin));
}

bool ETCController::update_state() {
    ETC_state.APPS1_voltage = APPS1_input.read() * 3.3f;
    ETC_state.APPS2_voltage = APPS2_input.read() * 3.3f;
    ETC_state.BPPS_voltage = BPPS_input.read() * 3.3f;
    ETC_state.front_BSE_voltage = front_BSE_input.read() * 3.0f;
    ETC_state.rear_BSE_voltage = rear_BSE_input.read() * 3.0f;
    ETC_state.RTD_button_active = RTD_button.read();

    ETC_state.APPS1_position = clamp((ETC_state.APPS1_voltage - APPS1_min_voltage) / (APPS1_max_voltage - APPS1_min_voltage));
    ETC_state.APPS2_position = clamp((ETC_state.APPS2_voltage - APPS2_min_voltage) / (APPS2_max_voltage - APPS2_min_voltage));
    ETC_state.BPPS_position = clamp((ETC_state.BPPS_voltage - BPPS_min_voltage) / (BPPS_max_voltage - BPPS_min_voltage));

    ETC_state.APPS_position_avg = (ETC_state.APPS1_position + ETC_state.APPS2_position) / 2.0f;

    if (ETC_state.RTD_state) {
        update_implaus();
    }
}



// CODE APPS / Brake PEdal Plausibiliuty Check AND change naming from TPS to Pedal sensors or similar. After that, write example in main.cpp for tomorrow (today) testing. Also add BPPS out of range specific plausbibility after, dontg combine. Last thing to add is ability to regen and profile.



void ETCController::update_implaus_timer(Timer &timer, bool &timer_running, float min_time, bool implaus_state, bool &etc_implaus) { 
    if (etc_implaus) { return; }

    if (implaus_state) {
        if (!timer_running) {
            timer.reset();
            timer.start();
            timer_running = true;
        } else {
            float time = std::chrono::duration<float>(timer.elapsed_time()).count();
            if (time > min_time) {
                etc_implaus = true;
                ETC_state.RTD_state = false;
                ETC_state.motor_enabled = false;

                timer.stop();
                timer.reset();
                timer_running = false;
            }
        }
    } else {
        if (timer_running) {
            timer.stop();
            timer.reset();
            timer_running = false;
        }
    }
}

void ETCController::update_implaus() {
    bool APPS_deviation_implaus = std::abs(ETC_state.APPS1_position - ETC_state.APPS2_position) > max_APPS_position_deviation;
    bool APPS_range_implaus = !in_range(ETC_state.APPS1_voltage, APPS1_min_voltage, APPS1_max_voltage, PS_voltage_margin) || !in_range(ETC_state.APPS2_voltage, APPS2_min_voltage, APPS2_max_voltage, PS_voltage_margin);
    bool BPPS_range_implaus = !in_range(ETC_state.BPPS_voltage, BPPS_min_voltage, BPPS_max_voltage, PS_voltage_margin);
    bool brake_and_accel_implaus = (ETC_state.front_BSE_voltage > front_BSE_activation_voltage || ETC_state.rear_BSE_voltage > rear_BSE_activation_voltage) && ETC_state.APPS_position_avg > 0.25f;

    update_implaus_timer(APPS_deviation_implaus_timer, APPS_deviation_implaus_timer_runnning, implaus_min_time, APPS_deviation_implaus, ETC_state.APPS_deviation_implaus);
    update_implaus_timer(APPS_range_implaus_timer, APPS_range_implaus_timer_running, implaus_min_time, APPS_range_implaus, ETC_state.APPS_range_implaus);
    update_implaus_timer(BPPS_range_implaus_timer, BPPS_range_implaus_timer_running, implaus_min_time, BPPS_range_implaus, ETC_state.BPPS_range_implaus);
    if (brake_and_accel_implaus) {
        ETC_state.brake_and_accel_implaus = true;
        ETC_state.motor_enabled = false;
    }
}

void ETCController::update_RTD(bool GLV_charged) {
    bool BSPD_fault = BSPD_fault_input.read();
    ETC_state.TS_active = GLV_charged && !BSPD_fault;

    if (ETC_state.brake_and_accel_implaus && ETC_state.APPS_position_avg < 0.05f) {
        ETC_state.brake_and_accel_implaus = false;
        
        if (ETC_state.RTD_state) {
            ETC_state.motor_enabled = true;
        }
    }

    if (!ETC_state.RTD_state && ETC_state.TS_active && (ETC_state.BPPS_position > BPPS_brake_engage_percent) && ETC_state.RTD_button_active) {
        ETC_state.RTD_state = true;
        ETC_state.motor_enabled = true;

        ETC_state.APPS_deviation_implaus = false;
        ETC_state.APPS_range_implaus = false;
        ETC_state.BPPS_range_implaus = false;
        ETC_state.brake_and_accel_implaus = false;
    } 
}

void ETCController::update_regen(float speed) {
    ETC_state.can_regen = in_range(speed, 0.0f, 5.0f, 0.0f);
    ETC_state.must_use_hydraulic_brakes = ETC_state.BPPS_position > BPPS_max_non_regen_braking;
}

void ETCController::toggle_RTD(bool new_RTD_state) {
    RTD_light.write(new_RTD_state);
    ETC_state.RTD_state = new_RTD_state;

    if (new_RTD_state) {
        RTD_buzzer.write(1);
        RTD_buzzer_timeout.attach([this]{ RTD_buzzer.write(0); }, RTD_buzzer_duration);
    } 
}

ETCState ETCController::get_ETC_state() {
    return ETC_state;
}
