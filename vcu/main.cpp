#include "Callback.h"
#include "EventQueue.h"
#include "PinNames.h"
#include "mbed.h"
#include "etc_controller.h"

EventQueue vcu_queue;
Thread queue_thread;

CAN can{PA_11, PA_12, 500000};
ETCController etc(PC_1, PC_2, PC_3, PA_1, PA_0, PC_13, PC_0, PA_7, PA_2);

void send_CAN_message();

int main() {
    printf("Hello World!!\n");

    vcu_queue.call_every(50ms, &send_CAN_message);
    queue_thread.start(callback(&vcu_queue, &EventQueue::dispatch_forever));

    CANMessage rx;
    while (true) {
        if (can.read(rx)) {
            switch (rx.id) {
                case 392: {
                    etc.GLV_ok = rx.data[0] & 0b00001000; // CHANGE TO WHATEVER BATT_STATUS_PRECHARGE_DONE IS
                    etc.shutdown_closed = rx.data[0] & 0b00001000; // CHANGE TO WHATEVER BATT_STATUS_SHUTDOWN_FINAL IS
                    break;
                }
                case 1154: {
                    uint16_t wheel_rpm = rx.data[0] | (rx.data[1] << 8); 
                    float wheel_radius = 0.190f;
                    float ground_speed = (11 / 40.0f) * ( 2 * M_PI * wheel_radius); // gear ratio * circumference
                    ground_speed *= wheel_rpm; // meters / minute
                    ground_speed *= 60.0f / 1000.0f; // km / hr

                    etc.update_regen(ground_speed);
                    break;
                }
            }
        }

        etc.update_state();
    }

    return 0;
}

void send_CAN_message() {
    const ETCState &etc_state = etc.state;

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
        (etc_state.implaus_APPS_deviation << 2) |
        (etc_state.implaus_APPS_range << 3) |
        (etc_state.implaus_BPPS_range << 4) |
        (etc_state.implaus_brake_and_accel << 5) |
        (etc_state.can_regen << 6) |
        (etc_state.must_use_hydraulic_brakes << 7);

    CANMessage msg0{393, buf0, 8};
    CANMessage msg1{394, buf1, 6};
    can.write(msg0);
    can.write(msg1);
}
