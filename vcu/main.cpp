#include "mbed.h"
#include "etc_controller.h"

EventQueue etc_queue;
EventQueue sme_queue;
EventQueue imu_queue;
Thread etc_queue_thread;
Thread sme_queue_thread;
Thread imu_queue_thread;

CAN canP{PB_8, PB_9, 500000};
CAN canD{PB_5, PB_6, 2000000}; // using both by sending the messages in parallel, but not sure if this is correct usage
ETCController etc{PC_1, PC_2, PC_3, PA_1, PA_0, PC_13, PC_0, PA_7, PB_1, PC_4, PC_12, PD_2};
const ETCState &etc_state = etc.state;

void send_etc_CAN_messages();
void send_sme_CAN_messages();
void send_imu_CAN_messages_fast();
void send_imu_CAN_messages_slow();

namespace {
constexpr float RAD_TO_DEG = 57.2957795f;
}

int main() {
    printf("Hello World!!\n");

    etc_queue.call_every(50ms, &send_etc_CAN_messages);
    sme_queue.call_every(40ms, &send_sme_CAN_messages);
    imu_queue.call_every(10ms, &send_imu_CAN_messages_fast); // 100Hz
    imu_queue.call_every(20ms, &send_imu_CAN_messages_slow); // 50Hz
    etc_queue_thread.start(callback(&etc_queue, &EventQueue::dispatch_forever));
    sme_queue_thread.start(callback(&sme_queue, &EventQueue::dispatch_forever));
    imu_queue_thread.start(callback(&imu_queue, &EventQueue::dispatch_forever));

    CANMessage rx;
    while (true) {
        if (canP.read(rx)) { // currently just reads from 1, but maybe make this both for possible optimization?
            switch (rx.id) {
                case 392: {
                    etc.battery_precharged = rx.data[0] & 0b00000010; 
                    etc.shutdown_closed = rx.data[0] & 0b00100000; 
                    break;
                }
                case 1154: {
                    uint16_t wheel_rpm = rx.data[0] | (rx.data[1] << 8); 
                    float wheel_radius = 0.190f;
                    float ground_speed = (11 / 40.0f) * ( 2 * M_PI * wheel_radius); // gear ratio * circumference
                    ground_speed *= wheel_rpm; // meters / minute
                    ground_speed *= 60.0f / 1000.0f; // km / hr

                    etc.update_regen_state(ground_speed);
                    break;
                }
            }
        }

        etc.update_state();
    }

    return 0;
}

void send_etc_CAN_messages() {
    uint8_t buf0[8];
    uint16_t APPS1_scaled_voltage = static_cast<uint16_t>(etc_state.APPS1_voltage * 1000);
    uint16_t APPS2_scaled_voltage = static_cast<uint16_t>(etc_state.APPS2_voltage * 1000);
    uint16_t BPPS_scaled_voltage = static_cast<uint16_t>(etc_state.BPPS_voltage * 1000);
    buf0[0] = APPS1_scaled_voltage & 0xFF;
    buf0[1] = APPS1_scaled_voltage >> 8;
    buf0[2] = APPS2_scaled_voltage & 0xFF;
    buf0[3] = APPS2_scaled_voltage >> 8;
    buf0[4] = BPPS_scaled_voltage & 0xFF;
    buf0[5] = BPPS_scaled_voltage >> 8;
    buf0[6] = static_cast<uint8_t>(etc_state.APPS_position_avg * 100);
    buf0[7] = static_cast<uint8_t>(etc_state.BPPS_position * 100);

    uint8_t buf1[8];
    uint16_t front_BSE_scaled_voltage = static_cast<uint16_t>(etc_state.front_BSE_voltage * 1000);
    uint16_t rear_BSE_scaled_voltage = static_cast<uint16_t>(etc_state.rear_BSE_voltage * 1000);
    buf1[0] = front_BSE_scaled_voltage & 0xFF;
    buf1[1] = front_BSE_scaled_voltage >> 8;
    buf1[2] = rear_BSE_scaled_voltage & 0xFF;
    buf1[3] = rear_BSE_scaled_voltage >> 8;
    buf1[4] = etc_state.ready_to_drive |
        (etc_state.motor_enabled << 1) |
        (etc_state.rtd_button_pressed << 2) |
        (etc.battery_precharged << 3) |
        (etc_state.implaus_APPS_range << 4) |
        (etc_state.implaus_BPPS_range << 5) |
        (etc_state.implaus_APPS_deviation << 6) |
        (etc_state.implaus_BSE_range << 7);
    buf1[5] = etc_state.implaus_brake_and_accel |
        (etc_state.reversing << 1) | // NEED REVERSE
        (etc_state.brakelight_enabled << 2) |
        (etc_state.can_regen << 3) |
        (etc_state.must_use_hydraulic_brakes << 4);

    CANMessage status1_msg{393, buf0, 8};
    CANMessage status2_msg{394, buf1, 6};
    canD.write(status1_msg);
    canD.write(status2_msg);
}

void send_sme_CAN_messages() {
    etc.update_mbb_alive();

    uint8_t buf0[8];
    buf0[0] = etc_state.motor_torque & 0xFF;
    buf0[1] = etc_state.motor_torque >> 8;
    buf0[2] = etc_state.MAX_SPEED & 0xFF;
    buf0[3] = etc_state.MAX_SPEED >> 8;
    buf0[4] = !etc_state.reversing |
        (etc_state.reversing << 1) |
        (etc_state.motor_enabled << 3);
    buf0[5] = etc_state.mbb_alive;

    uint8_t buf1[8];
    buf1[0] = etc_state.CHARGE_CURRENT_LIMIT & 0xFF;
    buf1[1] = etc_state.CHARGE_CURRENT_LIMIT >> 8;
    buf1[2] = etc_state.DISCHARGE_CURRENT_LIMIT & 0xFF;
    buf1[3] = etc_state.DISCHARGE_CURRENT_LIMIT >> 8;


    CANMessage throttle_msg{390, buf0, 8};
    CANMessage currents_msg{646, buf1, 8};

    canP.write(throttle_msg);
    canP.write(currents_msg);
}

void send_imu_CAN_messages_slow() {
    uint8_t buf_ypr[6];
    int16_t yaw   = static_cast<int16_t>(etc_state.vectornav.ypr.yaw   * 100);
    int16_t pitch = static_cast<int16_t>(etc_state.vectornav.ypr.pitch * 100);
    int16_t roll  = static_cast<int16_t>(etc_state.vectornav.ypr.roll  * 100);
    buf_ypr[0] = yaw & 0xFF;
    buf_ypr[1] = yaw >> 8;
    buf_ypr[2] = pitch & 0xFF;
    buf_ypr[3] = pitch >> 8;
    buf_ypr[4] = roll & 0xFF;
    buf_ypr[5] = roll >> 8;

    uint8_t buf_vel[6];
    int16_t vx = static_cast<int16_t>(etc_state.vectornav.velBody[0] * 100);
    int16_t vy = static_cast<int16_t>(etc_state.vectornav.velBody[1] * 100);
    int16_t vz = static_cast<int16_t>(etc_state.vectornav.velBody[2] * 100);
    buf_vel[0] = vx & 0xFF;
    buf_vel[1] = vx >> 8;
    buf_vel[2] = vy & 0xFF;
    buf_vel[3] = vy >> 8;
    buf_vel[4] = vz & 0xFF;
    buf_vel[5] = vz >> 8;

    uint8_t buf_latlon[8];
    int32_t lat = static_cast<int32_t>(etc_state.vectornav.pos.lat * 1e7);
    int32_t lon = static_cast<int32_t>(etc_state.vectornav.pos.lon * 1e7);
    buf_latlon[0] =  lat        & 0xFF;
    buf_latlon[1] = (lat >> 8)  & 0xFF;
    buf_latlon[2] = (lat >> 16) & 0xFF;
    buf_latlon[3] = (lat >> 24) & 0xFF;
    buf_latlon[4] =  lon        & 0xFF;
    buf_latlon[5] = (lon >> 8)  & 0xFF;
    buf_latlon[6] = (lon >> 16) & 0xFF;
    buf_latlon[7] = (lon >> 24) & 0xFF;

    uint8_t buf_alt[4];
    int32_t alt = static_cast<int32_t>(etc_state.vectornav.pos.alt * 1000);
    buf_alt[0] =  alt        & 0xFF;
    buf_alt[1] = (alt >> 8)  & 0xFF;
    buf_alt[2] = (alt >> 16) & 0xFF;
    buf_alt[3] = (alt >> 24) & 0xFF;

    CANMessage ypr_msg    {0x3D0, buf_ypr,    6};
    CANMessage vel_msg    {0x2D2, buf_vel,     6};
    CANMessage latlon_msg {0x2D1, buf_latlon,  8};
    CANMessage alt_msg    {0x3D2, buf_alt,     4};

    canD.write(ypr_msg);
    canD.write(vel_msg);
    canD.write(latlon_msg);
    canD.write(alt_msg);
}

void send_imu_CAN_messages_fast() {
    uint8_t buf_accel[6];
    int16_t ax = static_cast<int16_t>(etc_state.vectornav.accel[0] * 100);
    int16_t ay = static_cast<int16_t>(etc_state.vectornav.accel[1] * 100);
    int16_t az = static_cast<int16_t>(etc_state.vectornav.accel[2] * 100);
    buf_accel[0] = ax & 0xFF;
    buf_accel[1] = ax >> 8;
    buf_accel[2] = ay & 0xFF;
    buf_accel[3] = ay >> 8;
    buf_accel[4] = az & 0xFF;
    buf_accel[5] = az >> 8;

    uint8_t buf_gyro[6];
    int16_t wx = static_cast<int16_t>(etc_state.vectornav.angRate[0] * RAD_TO_DEG * 10);
    int16_t wy = static_cast<int16_t>(etc_state.vectornav.angRate[1] * RAD_TO_DEG * 10);
    int16_t wz = static_cast<int16_t>(etc_state.vectornav.angRate[2] * RAD_TO_DEG * 10);
    buf_gyro[0] = wx & 0xFF;
    buf_gyro[1] = wx >> 8;
    buf_gyro[2] = wy & 0xFF;
    buf_gyro[3] = wy >> 8;
    buf_gyro[4] = wz & 0xFF;
    buf_gyro[5] = wz >> 8;

    CANMessage accel_msg  {0x2D0, buf_accel,  6};
    CANMessage gyro_msg   {0x3D1, buf_gyro,   6};

    canD.write(accel_msg);
    canD.write(gyro_msg);
}   