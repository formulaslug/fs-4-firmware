#include "etc_controller.h"
#include "traction_control.h"
#include "mbed.h"

EventQueue etc_queue;
EventQueue imu_queue;
Thread etc_thread;
Thread imu_queue_thread;

AnalogIn steering_position{PC_5};
DigitalIn bspd_fault{PA_2};
DigitalIn bspd_shutdown_out{PA_3};

CAN canP{PB_8, PB_9, 500000};
CAN canD{
    PB_5, PB_6, 1000000
}; // using both by sending the messages in parallel, but not sure if this is correct usage
ETCController etc{PC_1, PC_2, PC_3, PA_1, PA_0, PC_13, PC_0, PA_7, PB_1, PC_4, PC_12, PD_2};
const ETCState& etc_state = etc.state;

void send_etc_CAN_messages();
void send_sme_CAN_messages();
void send_imu_CAN_messages_fast();
void send_imu_CAN_messages_slow();
void update_traction_control();

namespace {
constexpr float RAD_TO_DEG = 57.2957795f;
}

int main() {

    printf("Hello World!!\n");

    etc_queue.call_every(50ms, &send_etc_CAN_messages);
    etc_queue.call_every(40ms, &send_sme_CAN_messages);
    // imu_queue.call_every(10ms, &send_imu_CAN_messages_fast); // 100Hz
    // imu_queue.call_every(20ms, &send_imu_CAN_messages_slow); // 50Hz
    etc_thread.start(callback(&etc_queue, &EventQueue::dispatch_forever));
    // imu_queue_thread.start(callback(&imu_queue, &EventQueue::dispatch_forever));

    CANMessage rx;
    while (true) {
        if (canP.read(
                rx
            )) { // currently just reads from 1, but maybe make this both for possible optimization?
            switch (rx.id) {
            case 0x391: {
                etc.battery_precharged = rx.data[0] & 0b01000000;
                etc.shutdown_closed = rx.data[0] & 0b00000100;
                if (!etc.battery_precharged | !etc.shutdown_closed) {
                    etc.state.ready_to_drive = false;
                    etc.state.motor_enabled = false;
                    etc.rtd_light.write(false);
                }
                //printf("battery precharged: %d, shutdown closed: %d\n", etc.battery_precharged, etc.shutdown_closed);
                break;
            }
            case 1154: { // SME_TPDO_Torque_speed
                uint16_t wheel_rpm = rx.data[0] | (rx.data[1] << 8);
                float wheel_radius = 0.190f;
                float ground_speed =
                    (11 / 40.0f) * (2 * M_PI * wheel_radius); // gear ratio * circumference
                ground_speed *= wheel_rpm;                    // meters / minute
                ground_speed *= 60.0f / 1000.0f;              // km / hr

                etc.update_regen_state(ground_speed);
                break;
            }
            }
        }
        if (canD.read(rx)) {
          switch (rx.id) {
              case 421: {
                  etc.state.wheel_rpm_fl = (rx.data[0] + (rx.data[1] << 8)) * 0.1f;
                  break;
              }
              case 422: {
                  etc.state.wheel_rpm_fr = (rx.data[0] + (rx.data[1] << 8)) * 0.1f;
                  break;
              }
              case 423: {
                  etc.state.wheel_rpm_bl = (rx.data[0] + (rx.data[1] << 8)) * 0.1f;
                  break;
              }
              case 424: {
                  etc.state.wheel_rpm_br = (rx.data[0] + (rx.data[1] << 8)) * 0.1f;
                  break;
              }
              case 432: {
                  const uint8_t modes = rx.data[0];
                  etc.state.drive_mode = modes & 0b00000011;
                  etc.state.traction_mode = (modes >> 2) & 0b00000011;
                  etc.state.regen_mode = (modes >> 4) & 0b00000011;
              }
            }
        }

        etc.update_state();
    }

    return 0;
}

void send_etc_CAN_messages() {
    uint8_t tpdo_pedal_travel[8] = {0};
    uint16_t APPS1_scaled_voltage = static_cast<uint16_t>(etc_state.APPS1_voltage * 1000);
    uint16_t APPS2_scaled_voltage = static_cast<uint16_t>(etc_state.APPS2_voltage * 1000);
    uint16_t BPPS_scaled_voltage = static_cast<uint16_t>(etc_state.BPPS_voltage * 1000);
    tpdo_pedal_travel[0] = APPS1_scaled_voltage & 0xFF;
    tpdo_pedal_travel[1] = APPS1_scaled_voltage >> 8;
    tpdo_pedal_travel[2] = APPS2_scaled_voltage & 0xFF;
    tpdo_pedal_travel[3] = APPS2_scaled_voltage >> 8;
    tpdo_pedal_travel[4] = BPPS_scaled_voltage & 0xFF;
    tpdo_pedal_travel[5] = BPPS_scaled_voltage >> 8;
    tpdo_pedal_travel[6] = static_cast<uint8_t>(etc_state.APPS_position_avg * 100);
    tpdo_pedal_travel[7] = static_cast<uint8_t>(etc_state.BPPS_position * 100);

    uint8_t tpdo_status[8] = {0};
    uint16_t front_BSE_scaled_voltage = static_cast<uint16_t>(etc_state.front_BSE_voltage * 1000);
    uint16_t rear_BSE_scaled_voltage = static_cast<uint16_t>(etc_state.rear_BSE_voltage * 1000);
    uint16_t steering_position_mv = steering_position.read_voltage() * 1000;

    tpdo_status[0] = etc_state.ready_to_drive
                     | (etc_state.motor_enabled << 1)
                     | (etc_state.rtd_button_pressed << 2)
                     | (etc.battery_precharged << 3)
                     | (etc_state.implaus_APPS_range << 4)
                     | (etc_state.implaus_BPPS_range << 5)
                     | (etc_state.implaus_APPS_deviation << 6)
                     | (etc_state.implaus_BSE_range << 7);
    tpdo_status[1] = etc_state.implaus_brake_and_accel
                     | (etc_state.reversing << 1)
                     | (etc_state.brakelight_enabled << 2)
                     | (etc_state.regen_allowed << 3)
                     | (etc_state.solenoid_open << 4)
                     | (bspd_fault.read() << 6)
                     | (bspd_shutdown_out.read() << 7);
    tpdo_status[2] = front_BSE_scaled_voltage & 0xFF;
    tpdo_status[3] = front_BSE_scaled_voltage >> 8;
    tpdo_status[4] = rear_BSE_scaled_voltage & 0xFF;
    tpdo_status[5] = rear_BSE_scaled_voltage >> 8;
    tpdo_status[6] = steering_position_mv & 0xFF;
    tpdo_status[7] = steering_position_mv >> 8;

    CANMessage msg1{402, tpdo_pedal_travel, 8};
    CANMessage msg2{403, tpdo_status, 8};
    canP.write(msg1);
    canP.write(msg2);
}

void send_sme_CAN_messages() {
    etc.update_mbb_alive();

    update_traction_control();
    if (etc_state.traction_mode != 0) {
      const float tc_torque_reduction_factor = etc.traction_controller.get_output();
      etc.state.motor_torque = static_cast<int16_t>(etc.state.motor_torque * tc_torque_reduction_factor);
    }

    uint8_t tpdo_throttle_demand[8];
    tpdo_throttle_demand[0] = etc_state.motor_torque & 0xFF;
    tpdo_throttle_demand[1] = etc_state.motor_torque >> 8;
    tpdo_throttle_demand[2] = etc_state.MAX_SPEED & 0xFF;
    tpdo_throttle_demand[3] = etc_state.MAX_SPEED >> 8;
    tpdo_throttle_demand[4] =
        !etc_state.reversing | (etc_state.reversing << 1) | (etc_state.motor_enabled << 3);
    tpdo_throttle_demand[5] = etc_state.mbb_alive;

    uint8_t tpdo_max_currents[8];
    tpdo_max_currents[0] = etc_state.CHARGE_CURRENT_LIMIT & 0xFF;
    tpdo_max_currents[1] = etc_state.CHARGE_CURRENT_LIMIT >> 8;
    tpdo_max_currents[2] = etc_state.DISCHARGE_CURRENT_LIMIT & 0xFF;
    tpdo_max_currents[3] = etc_state.DISCHARGE_CURRENT_LIMIT >> 8;

    uint8_t tpdo_traction_data[8];
    uint8_t tc_slip = static_cast<uint8_t>(etc.traction_controller.get_slip() * 100.0f);
    uint8_t tc_output = static_cast<uint8_t>(etc.traction_controller.get_output() * 100.0f);
    uint8_t tc_integral = static_cast<uint8_t>(etc.traction_controller.get_integral() * 100.0f);
    uint8_t tc_loop_time = static_cast<uint8_t>(etc.traction_controller.get_loop_time() * 1000.0f);
    int16_t tc_raw_derivative = static_cast<int16_t>(etc.traction_controller.get_raw_derivative() * 1000.0f);
    int16_t tc_smoothed_derivative = static_cast<int16_t>(etc.traction_controller.get_smoothed_derivative() * 1000.0f);
    tpdo_traction_data[0] = tc_slip;
    tpdo_traction_data[1] = tc_output;
    tpdo_traction_data[2] = tc_integral;
    tpdo_traction_data[3] = tc_loop_time;
    tpdo_traction_data[4] = tc_raw_derivative & 0xFF;
    tpdo_traction_data[5] = tc_raw_derivative >> 8;
    tpdo_traction_data[6] = tc_smoothed_derivative & 0xFF;
    tpdo_traction_data[7] = tc_smoothed_derivative >> 8;

    CANMessage throttle_msg{390, tpdo_throttle_demand, 8};
    CANMessage currents_msg{646, tpdo_max_currents, 8};
    CANMessage traction_msg{660, tpdo_traction_data, 8};

    canP.write(throttle_msg);
    canP.write(currents_msg);
    canD.write(traction_msg);
}

void update_traction_control() {
  etc.traction_controller.update(etc.state.wheel_rpm_fl, etc.state.wheel_rpm_fr, etc.state.wheel_rpm_bl, etc.state.wheel_rpm_br);
}

void send_imu_CAN_messages_slow() {
    // Nathaniel wanted them all fast ¯\_(ツ)_/¯
}

/// 100Hz VectorNav Messages
// void send_imu_CAN_messages_fast() {
//     uint8_t buf_accel[6];
//     int16_t ax = static_cast<int16_t>(etc_state.vectornav.accelBody[0] * 100);
//     int16_t ay = static_cast<int16_t>(etc_state.vectornav.accelBody[1] * 100);
//     int16_t az = static_cast<int16_t>(etc_state.vectornav.accelBody[2] * 100);
//     buf_accel[0] = ax & 0xFF;
//     buf_accel[1] = ax >> 8;
//     buf_accel[2] = ay & 0xFF;
//     buf_accel[3] = ay >> 8;
//     buf_accel[4] = az & 0xFF;
//     buf_accel[5] = az >> 8;
//
//     uint8_t buf_gyro[6];
//     int16_t wx = static_cast<int16_t>(etc_state.vectornav.angRate[0] * RAD_TO_DEG * 10);
//     int16_t wy = static_cast<int16_t>(etc_state.vectornav.angRate[1] * RAD_TO_DEG * 10);
//     int16_t wz = static_cast<int16_t>(etc_state.vectornav.angRate[2] * RAD_TO_DEG * 10);
//     buf_gyro[0] = wx & 0xFF;
//     buf_gyro[1] = wx >> 8;
//     buf_gyro[2] = wy & 0xFF;
//     buf_gyro[3] = wy >> 8;
//     buf_gyro[4] = wz & 0xFF;
//     buf_gyro[5] = wz >> 8;
//
//     uint8_t buf_ypr[6];
//     int16_t yaw = static_cast<int16_t>(etc_state.vectornav.ypr.yaw * 100);
//     int16_t pitch = static_cast<int16_t>(etc_state.vectornav.ypr.pitch * 100);
//     int16_t roll = static_cast<int16_t>(etc_state.vectornav.ypr.roll * 100);
//     buf_ypr[0] = yaw & 0xFF;
//     buf_ypr[1] = yaw >> 8;
//     buf_ypr[2] = pitch & 0xFF;
//     buf_ypr[3] = pitch >> 8;
//     buf_ypr[4] = roll & 0xFF;
//     buf_ypr[5] = roll >> 8;
//
//     uint8_t buf_vel[6];
//     int16_t vx = static_cast<int16_t>(etc_state.vectornav.velBody[0] * 100);
//     int16_t vy = static_cast<int16_t>(etc_state.vectornav.velBody[1] * 100);
//     int16_t vz = static_cast<int16_t>(etc_state.vectornav.velBody[2] * 100);
//     buf_vel[0] = vx & 0xFF;
//     buf_vel[1] = vx >> 8;
//     buf_vel[2] = vy & 0xFF;
//     buf_vel[3] = vy >> 8;
//     buf_vel[4] = vz & 0xFF;
//     buf_vel[5] = vz >> 8;
//
//     uint8_t buf_latlon[8];
//     int32_t lat = static_cast<int32_t>(etc_state.vectornav.pos.lat * 1e7);
//     int32_t lon = static_cast<int32_t>(etc_state.vectornav.pos.lon * 1e7);
//     buf_latlon[0] = lat & 0xFF;
//     buf_latlon[1] = (lat >> 8) & 0xFF;
//     buf_latlon[2] = (lat >> 16) & 0xFF;
//     buf_latlon[3] = (lat >> 24) & 0xFF;
//     buf_latlon[4] = lon & 0xFF;
//     buf_latlon[5] = (lon >> 8) & 0xFF;
//     buf_latlon[6] = (lon >> 16) & 0xFF;
//     buf_latlon[7] = (lon >> 24) & 0xFF;
//
//     uint8_t buf_alt[4];
//     int32_t alt = static_cast<int32_t>(etc_state.vectornav.pos.alt * 1000);
//     buf_alt[0] =  alt        & 0xFF;
//     buf_alt[1] = (alt >> 8)  & 0xFF;
//     buf_alt[2] = (alt >> 16) & 0xFF;
//     buf_alt[3] = (alt >> 24) & 0xFF;
//
//     CANMessage ypr_msg    {0x3D0, buf_ypr,    6};
//     CANMessage vel_msg    {0x2D2, buf_vel,     6};
//     CANMessage latlon_msg {0x2D1, buf_latlon,  8};
//     CANMessage alt_msg    {0x3D2, buf_alt,     4};
//
//     canD.write(ypr_msg);
//     canD.write(vel_msg);
//     canD.write(latlon_msg);
//     canD.write(alt_msg);
// }

// void send_imu_CAN_messages_fast() {
//     uint8_t buf_accel[6];
//     int16_t ax = static_cast<int16_t>(etc_state.vectornav.accelBody[0] * 100);
//     int16_t ay = static_cast<int16_t>(etc_state.vectornav.accelBody[1] * 100);
//     int16_t az = static_cast<int16_t>(etc_state.vectornav.accelBody[2] * 100);
//     buf_accel[0] = ax & 0xFF;
//     buf_accel[1] = ax >> 8;
//     buf_accel[2] = ay & 0xFF;
//     buf_accel[3] = ay >> 8;
//     buf_accel[4] = az & 0xFF;
//     buf_accel[5] = az >> 8;
//
//    uint8_t buf_alt[4];
//     int32_t alt = static_cast<int32_t>(etc_state.vectornav.pos.alt * 1000);
//     buf_alt[0] = alt & 0xFF;
//     buf_alt[1] = (alt >> 8) & 0xFF;
//     buf_alt[2] = (alt >> 16) & 0xFF;
//     buf_alt[3] = (alt >> 24) & 0xFF;
//
//     CANMessage ypr_msg{0x3D0, buf_ypr, 6};
//     CANMessage vel_msg{0x2D2, buf_vel, 6};
//     CANMessage latlon_msg{0x2D1, buf_latlon, 8};
//     CANMessage alt_msg{0x3D2, buf_alt, 4};
//
//     // canD.write(ypr_msg);
//     // canD.write(vel_msg);
//     // canD.write(latlon_msg);
//     // canD.write(alt_msg);
// }

// void send_imu_CAN_messages_fast() {
//     uint8_t buf_accel[6];
//     int16_t ax = static_cast<int16_t>(etc_state.vectornav.accelBody[0] * 100);
//     int16_t ay = static_cast<int16_t>(etc_state.vectornav.accelBody[1] * 100);
//     int16_t az = static_cast<int16_t>(etc_state.vectornav.accelBody[2] * 100);
//     buf_accel[0] = ax & 0xFF;
//     buf_accel[1] = ax >> 8;
//     buf_accel[2] = ay & 0xFF;
//     buf_accel[3] = ay >> 8;
//     buf_accel[4] = az & 0xFF;
//     buf_accel[5] = az >> 8;
//
//     uint8_t buf_gyro[6];
//     int16_t wx = static_cast<int16_t>(etc_state.vectornav.angRate[0] * RAD_TO_DEG * 10);
//     int16_t wy = static_cast<int16_t>(etc_state.vectornav.angRate[1] * RAD_TO_DEG * 10);
//     int16_t wz = static_cast<int16_t>(etc_state.vectornav.angRate[2] * RAD_TO_DEG * 10);
//     buf_gyro[0] = wx & 0xFF;
//     buf_gyro[1] = wx >> 8;
//     buf_gyro[2] = wy & 0xFF;
//     buf_gyro[3] = wy >> 8;
//     buf_gyro[4] = wz & 0xFF;
//     buf_gyro[5] = wz >> 8;
//
//     // VCU_VN_BODY_ACCEL
//     CANMessage accel_msg  {0x2D0, buf_accel,  6};
//     // VCU_VN_BODY_GYRO
//     CANMessage gyro_msg   {0x3D1, buf_gyro,   6};
//     // VCU_VN_YPR
//     CANMessage ypr_msg    {0x3D0, buf_ypr,    6};
//     // VCU_VN_BODY_VEL
//     CANMessage vel_msg    {0x2D2, buf_vel,     6};
//     // VCU_VN_LATLON
//     CANMessage latlon_msg {0x2D1, buf_latlon,  8};
//     CANMessage accel_msg  {0x2D0, buf_accel,  6};
//     CANMessage gyro_msg   {0x3D1, buf_gyro,   6};
//
//     canD.write(accel_msg);
//     canD.write(gyro_msg);
//     canD.write(ypr_msg);
//     canD.write(vel_msg);
//     canD.write(latlon_msg);
// }

void send_sync() { canP.write(CANMessage{0x80, (uint8_t*)nullptr, 0}); }
