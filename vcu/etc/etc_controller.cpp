//
// Created by Jackson Pinsonneault on 3/24/26.
//

#include "etc_controller.h"
#include "../imu/vectornav_imu.h"

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
    PinName brakelight_pin,
    PinName vectornav_tx,
    PinName vectornav_rx
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
      brakelight(brakelight_pin),
      vn_imu(vectornav_tx, vectornav_rx) {
    rtd_light.write(0);
    rtd_buzzer.write(0);
    solenoid.write(0);
    brakelight.write(0);

    rtd_button.rise(callback(this, &ETCController::rtd_button_irq));

    // spawns new thread for imu listener
    // CHECK_VN_ERR(vn_imu.connect());
    // CHECK_VN_ERR(vn_imu.init());
}

float ETCController::clamp(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

bool ETCController::in_range(float value, float low, float high) {
    return (value >= low) && (value <= high);
}

float ETCController::current_limit(float voltage, float current){
    float R0 = 0.008f / 19.0f;
    float limit = -1.0f / R0 * (2.5 - (voltage - 0.5f) - current * R0);
    if (limit > state.MAX_DISCHARGE_CURRENT_LIMIT) {
        return state.MAX_DISCHARGE_CURRENT_LIMIT;
    }
    return limit;
}

float ETCController::accelerator_mapping(float position) {
    // Tune these. Raising the scale of region 1 and 3 will make the pedal more sensetive in those regions.
    // Making them both 1 will return the pedal to normal feel.
    float region1 = 0.3f; // This represents the size of the region, not the position. 
    float region1_scale = 0.25f; 
    float region2 = 0.4f; // This represents the size of the region, not the position.
    float region3_scale = 0.25f;

    float region3 = 1.0f - region2 - region1;
    float region1_size = region1 * region1_scale; // This is the scaled size of the region
    float region3_size = region3 * region3_scale; // This is the scaled size of the region

    float region2_size = 1.0f - region1_size - region3_size;
    float region2_scale = region2_size / region2;

    if (position < 0.3f) {
        return position * region1_scale;
    };
    if (position < 0.7f) {
        return region1_size + (position - region1) * region2_scale;
    };
    return region1_size + region2_size + (position - region2) * region3_scale;
}

void ETCController::update_state() {
    state.APPS1_voltage = APPS1_input.read_voltage();
    state.APPS2_voltage = APPS2_input.read_voltage();
    state.BPPS_voltage = BPPS_input.read_voltage();
    state.front_BSE_voltage = front_BSE_input.read_voltage();
    state.rear_BSE_voltage = rear_BSE_input.read_voltage();

    state.APPS1_position = clamp(
        (state.APPS1_voltage - APPS1_MIN_VOLTAGE) / (APPS1_MAX_VOLTAGE - APPS1_MIN_VOLTAGE)
    );
    state.APPS2_position = clamp(
        (state.APPS2_voltage - APPS2_MIN_VOLTAGE) / (APPS2_MAX_VOLTAGE - APPS2_MIN_VOLTAGE)
    );
    state.BPPS_position =
        clamp((state.BPPS_voltage - BPPS_MIN_VOLTAGE) / (BPPS_MAX_VOLTAGE - BPPS_MIN_VOLTAGE));
    state.APPS_position_avg = (state.APPS1_position + state.APPS2_position) / 2.0f;
    
    update_implaus();

    state.APPS_position_avg = accelerator_mapping(state.APPS_position_avg);

    if (!REGEN_FORCE_DISABLE && state.regen_mode != 0) {
        state.unfiltered_motor_torque = static_cast<int16_t>(state.APPS_position_avg * MAX_TORQUE) - static_cast<int16_t>(state.BPPS_position * MAX_REGEN_TORQUE);
    } else {
        state.unfiltered_motor_torque = static_cast<int16_t>(state.APPS_position_avg * MAX_TORQUE);
    }

    if (!TRACTION_CONTROL_FORCE_DISABLE && state.traction_mode != 0 && state.motor_torque.read() > 0) {
      state.unfiltered_motor_torque = static_cast<int16_t>(state.motor_torque.read() * state.tc_torque_reduction_factor);
    }

    state.motor_torque.sample(state.unfiltered_motor_torque); // smooth out motor torque

    state.brakelight_enabled = state.motor_torque.read() < 0 || (state.BPPS_position > BPPS_BRAKE_ENGAGE_PERCENT);
    brakelight.write(state.brakelight_enabled);

    state.solenoid_open = SOLENOID_FORCE_OPEN ? true : state.regen_allowed;
    solenoid.write(state.solenoid_open);

    state.rtd_button_pressed = rtd_button.read();

    // vn_imu.refreshDataToState(state.vectornav);
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
        !in_range(state.APPS1_voltage, APPS1_MIN_VOLTAGE, APPS1_MAX_VOLTAGE)
        || !in_range(state.APPS2_voltage, APPS2_MIN_VOLTAGE, APPS2_MAX_VOLTAGE);
    bool implaus_BPPS_range = !in_range(state.BPPS_voltage, BPPS_MIN_VOLTAGE, BPPS_MAX_VOLTAGE);
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
        !in_range(speed, 0.0f, 5.0f) || state.BPPS_position > BPPS_MAX_NON_REGEN_BRAKING;
}

// todo: this function is not called?
void ETCController::set_regen_torque(bool is_regening, bool solenoid_open, int16_t regen_torque) {
    // state.is_regening = is_regening;
    state.solenoid_open = solenoid_open;
    // state.regen_torque = is_regening ? regen_torque : 0.0f;
}

void ETCController::update_mbb_alive() {
    state.mbb_alive++;
    state.mbb_alive %= 16;
}
