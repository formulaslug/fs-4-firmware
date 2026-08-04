//
// Created by Jackson Pinsonneault on 3/24/26.
//

#include "etc_controller.h"
#include "../imu/vectornav_imu.h"
#include <sys/stat.h>

ETCController::ETCController(
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
    // VectorNavIMU &vn_imu
)
    : unfiltered_APPS1_input(APPS1_pin),
      APPS1_input(unfiltered_APPS1_input, 60),
      unfiltered_APPS2_input(APPS2_pin),
      APPS2_input(unfiltered_APPS2_input, 60),
      unfiltered_BPPS_input(BPPS_pin),
      BPPS_input(unfiltered_BPPS_input, 60),
      unfiltered_front_BSE_input(front_BSE_pin),
      front_BSE_input(unfiltered_front_BSE_input, 60),
      unfiltered_rear_BSE_input(rear_BSE_pin),
      rear_BSE_input(unfiltered_rear_BSE_input, 60),
      rtd_button(rtd_button_pin),
      rtd_light(rtd_light_pin),
      rtd_buzzer(rtd_buzzer_pin),
      solenoid(solenoid_pin),
      brakelight(brakelight_pin) {
      // vn_imu(vn_imu) {
    rtd_light.write(0);
    rtd_buzzer.write(0);
    solenoid.write(0);
    brakelight.write(0);

    rtd_button.rise(callback(this, &ETCController::rtd_button_irq));
}

float ETCController::clamp(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

bool ETCController::in_range(float value, float low, float high) {
    return (value >= low) && (value <= high);
}

float ETCController::accelerator_mapping(float pedal_travel) {
    float region1 = 0.3f;
    float region1_scale = 2.0f;
    float region2 = 0.4f;
    float region3_scale = 2.0f;

    float region3 = 1 - region1 - region2;
    float scaled_region1 = region1 / region1_scale;
    float scaled_region3 = region3 / region3_scale;
    float scaled_region2 = 1 - scaled_region1 - scaled_region3;
    float region2_scale = region2 / scaled_region2;

    if (pedal_travel < scaled_region1) {
        return pedal_travel * region1_scale;
    }
    if (pedal_travel < scaled_region2 + scaled_region1) {
        float p1 = scaled_region1;
        float p1_power = p1 * region1_scale;
        float p2 = pedal_travel - scaled_region1;
        float p2_power = p2 * region2_scale;
        return p1_power + p2_power;
    }
    float p1 = scaled_region1;
    float p1_power = p1 * region1_scale;
    float p2 = scaled_region2;
    float p2_power = p2 * region2_scale;
    float p3 = pedal_travel - scaled_region1 - scaled_region2;
    float p3_power = p3 * region3_scale;
    return p1_power + p2_power + p3_power;
}

void ETCController::update_state() {
    state.APPS1_voltage = APPS1_input.read_voltage();
    state.APPS2_voltage = APPS2_input.read_voltage();
    state.BPPS_voltage = BPPS_input.read_voltage();
    state.front_BSE_voltage = front_BSE_input.read_voltage();
    state.rear_BSE_voltage = rear_BSE_input.read_voltage();

    state.front_BSE_pressure = ((state.front_BSE_voltage * 1000.0f - 330.0f) / (3300.0f - 660.0f)) * 2000.0f;
    state.read_BSE_pressure = ((state.rear_BSE_voltage * 1000.0f - 330.0f) / (3300.0f - 660.0f)) * 2000.0f;

    state.APPS1_position = clamp(
        (state.APPS1_voltage - state.APPS1_MIN_VOLTAGE) / (state.APPS1_MAX_VOLTAGE - state.APPS1_MIN_VOLTAGE)
    );
    state.APPS2_position = clamp(
        (state.APPS2_voltage - state.APPS2_MIN_VOLTAGE) / (state.APPS2_MAX_VOLTAGE - state.APPS2_MIN_VOLTAGE)
    );
    state.BPPS_position =
        clamp((state.BPPS_voltage - state.BPPS_MIN_VOLTAGE) / (state.BPPS_MAX_VOLTAGE - state.BPPS_MIN_VOLTAGE));
    state.APPS_position_avg = (state.APPS1_position + state.APPS2_position) / 2.0f;

    update_implaus();

    state.APPS_position_avg = accelerator_mapping(state.APPS_position_avg);

    if (!REGEN_FORCE_DISABLE && state.regen_mode != 0) {
        state.unfiltered_motor_torque = static_cast<int16_t>(state.APPS_position_avg * MAX_TORQUE) - static_cast<int16_t>(state.BPPS_position * MAX_REGEN_TORQUE);
    } else {
        state.unfiltered_motor_torque = static_cast<int16_t>(state.APPS_position_avg * MAX_TORQUE);
    }

    // if (!TRACTION_CONTROL_FORCE_DISABLE && state.traction_mode != 0 && state.motor_torque.read() > 0) {
    //   // state.unfiltered_motor_torque = static_cast<int16_t>(state.motor_torque.read() * state.tc_torque_reduction_factor);
    // }

    // state.motor_torque.sample(state.unfiltered_motor_torque); // smooth out motor torque

    state.brakelight_enabled = state.unfiltered_motor_torque < 0 || (state.BPPS_position > BPPS_BRAKE_ENGAGE_PERCENT);
    brakelight.write(state.brakelight_enabled);

    state.solenoid_open = SOLENOID_FORCE_OPEN ? true : state.regen_allowed;
    solenoid.write(!state.solenoid_open);

    state.rtd_button_pressed = rtd_button.read();
}

void ETCController::update_implaus_timer(
    Timer& timer, bool& timer_running, bool implaus_state, bool& etc_implaus
) {
    if (implaus_state) {
        if (etc_implaus) {
            return;
        }

        if (!timer_running) {
            timer.reset();
            timer.start();
            timer_running = true;
        } else {
            uint16_t time_ms_elapsed = timer.elapsed_time().count() / 1000;
            // Needs to have faulted for at least 100ms before the motor is disabled
            if (time_ms_elapsed > 100) {
                etc_implaus = true;

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
    bool implaus_APPS_deviation =
        std::abs(state.APPS1_position - state.APPS2_position) > MAX_APPS_POSITION_DEVIATION;
    bool implaus_APPS_range =
        !in_range(state.APPS1_voltage, state.APPS1_MIN_VOLTAGE, state.APPS1_MAX_VOLTAGE)
        || !in_range(state.APPS2_voltage, state.APPS2_MIN_VOLTAGE, state.APPS2_MAX_VOLTAGE);
    bool implaus_BPPS_range = !in_range(state.BPPS_voltage, state.BPPS_MIN_VOLTAGE, state.BPPS_MAX_VOLTAGE);
    bool implaus_BSE_range =
        !in_range(state.front_BSE_voltage, FRONT_BSE_MIN_VOLTAGE, FRONT_BSE_MAX_VOLTAGE)
        || !in_range(state.rear_BSE_voltage, REAR_BSE_MIN_VOLTAGE, REAR_BSE_MAX_VOLTAGE);
    // APPS / Brake Pedal Plausibility Check:
    // "With accelerator > 25%, press brake pedal. Axle MUST stop"
    // Note: brake pedal range is up for interpretation
    bool implaus_brake_and_accel = (state.BPPS_position > 0.25) && state.APPS_position_avg > 0.25f;

    update_implaus_timer(
        implaus_APPS_deviation_timer,
        implaus_APPS_deviation_timer_running,
        implaus_APPS_deviation,
        state.implaus_APPS_deviation
    );
    update_implaus_timer(
        implaus_APPS_range_timer,
        implaus_APPS_range_timer_running,
        implaus_APPS_range,
        state.implaus_APPS_range
    );
    update_implaus_timer(
        implaus_BPPS_range_timer,
        implaus_BPPS_range_timer_running,
        implaus_BPPS_range,
        state.implaus_BPPS_range
    );
    update_implaus_timer(
        implaus_BSE_range_timer,
        implaus_BSE_range_timer_running,
        implaus_BSE_range,
        state.implaus_BSE_range
    );

    // "axle may turn again once < 5% pedal position."
    if (state.implaus_brake_and_accel && state.APPS_position_avg < 0.05f) {
        state.implaus_brake_and_accel = false;
    }
    if (implaus_brake_and_accel) {
        state.implaus_brake_and_accel = true;
    }

    if (state.implaus_APPS_deviation
        || state.implaus_APPS_range
        || state.implaus_BPPS_range
        || state.implaus_brake_and_accel
        || state.implaus_BSE_range
        || !state.ready_to_drive)
    {
        state.motor_enabled = false;
    } else {
        state.motor_enabled = true;
    }
}

// Called in the rise irq for rtd_button
void ETCController::rtd_button_irq() {
    // TS_READY is battery CAN messages saying that precharge is done and
    // shutdown closed
    bool ts_ready = battery_precharged && shutdown_closed;
    bool rtd_condition = state.BPPS_position > BPPS_BRAKE_ENGAGE_PERCENT;
    if (!state.ready_to_drive && ts_ready && rtd_condition) {
        turn_on_rtd();
    } else {
        turn_off_rtd();
    }
}
void ETCController::turn_on_rtd() {
    state.ready_to_drive = true;
    rtd_light.write(1);
    rtd_buzzer.write(1);
    rtd_buzzer_timeout.attach([this] { rtd_buzzer.write(0); }, RTD_BUZZER_DURATION);
}
void ETCController::turn_off_rtd() {
    state.ready_to_drive = false;
    rtd_light.write(0);
}

void ETCController::update_regen_state(float speed) {
    state.regen_allowed =
        (!in_range(speed, 0.0f, 5.0f) || state.BPPS_position > BPPS_MAX_NON_REGEN_BRAKING) && !REGEN_FORCE_DISABLE;
}

// todo: this function is not called?
void ETCController::set_regen_torque(bool is_regening, bool solenoid_open, int16_t regen_torque) {
    // state.is_regening = is_regening;
    // state.solenoid_open = solenoid_open;
    // state.regen_torque = is_regening ? regen_torque : 0.0f;
}

float ETCController::current_limit(float voltage, float current) {
    float R0 = 0.00686f / 19.0f;
    float limit = -1.0f / R0 * (2.5 - (voltage - 0.5f) - current * R0);
    if (limit > state.MAX_DISCHARGE_CURRENT_LIMIT) {
        return state.MAX_DISCHARGE_CURRENT_LIMIT;
    }
    return limit;
}

void ETCController::update_mbb_alive() {
    state.mbb_alive++;
    state.mbb_alive %= 16;
}
