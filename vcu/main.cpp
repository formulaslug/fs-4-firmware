#include "mbed.h"
#include "etc_controller.h"

EventQueue etc_queue;
EventQueue sme_queue;
Thread etc_queue_thread;
Thread sme_queue_thread;

CAN can1{PB_8, PB_9, 500000};
CAN can2{PB_5, PB_6, 500000}; // using both by sending the messages in parallel, but not sure if this is correct usage
ETCController etc{PC_1, PC_2, PC_3, PA_1, PA_0, PC_13, PC_0, PA_7, PB_1, PC_4};
const ETCState &etc_state = etc.state;

void send_etc_CAN_messages();
void send_sme_CAN_messages();

int main() {
    printf("Hello World!!\n");

    etc_queue.call_every(50ms, &send_etc_CAN_messages);
    sme_queue.call_every(40ms, &send_sme_CAN_messages);
    etc_queue_thread.start(callback(&etc_queue, &EventQueue::dispatch_forever));
    sme_queue_thread.start(callback(&sme_queue, &EventQueue::dispatch_forever));

    CANMessage rx;
    while (true) {
        if (can1.read(rx)) { // currently just reads from 1, but maybe make this both for possible optimization?
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
    can1.write(status1_msg);
    can2.write(status2_msg);
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
    can1.write(throttle_msg);
    can2.write(currents_msg);
}
