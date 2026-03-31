#include "PinNames.h"
#include "mbed.h"
#include "etc_controller.h"

float VCU_can_frequency = 50.0f; // in hz
Timeout VCU_can_timeout;

CAN can{PA_11, PA_12, 500000};
ETCController ETC(PC_1, PC_2, PC_3, PA_1, PA_0, PC_13, PC_0, PA_7, PA_2);

void send_CAN_message();

int main() {
    printf("Hello World!!\n");

    VCU_can_timeout.attach(&send_CAN_message, 1000ms / VCU_can_frequency);

    CANMessage rx;
    while (true) {
        if (can.read(rx)) {
            switch (rx.id) {
                case 392: {
                    bool is_charged = rx.data[0] & 0b00001000; // checking if BATT_STATUS_PRECHARGE_DONE is true
                    ETC.update_RTD(is_charged);
                    break;
                }
                case 655361: {
                    float gps_speed = (rx.data[0] << 8) | rx.data[1]; // in mph, unscaled
                    gps_speed /= 100.0f; // scaling to 0.01 
                    gps_speed *= 1.60934f; // mph -> km/hr

                    ETC.update_regen(gps_speed);
                    break;
                }
            }
        }

        ETC.update_state();
    }

    return 0;
}

void send_CAN_message() {    
    ETCState ETC_state = ETC.get_ETC_state();

    uint8_t buf0[8];
    uint16_t APPS1_scaled_voltage = static_cast<uint16_t>(ETC_state.APPS1_voltage * 1000);
    uint16_t APPS2_scaled_voltage = static_cast<uint16_t>(ETC_state.APPS2_voltage * 1000);
    uint16_t BPPS_scaled_voltage = static_cast<uint16_t>(ETC_state.BPPS_voltage * 1000);
    buf0[0] = APPS1_scaled_voltage & 0b11111111; // first 8 bits of APPS1_voltage
    buf0[1] = APPS1_scaled_voltage >> 8; // last 8 bits of APPS1_voltage
    buf0[2] = APPS2_scaled_voltage & 0b11111111; // first 8 bits of APPS2_voltage
    buf0[3] = APPS2_scaled_voltage >> 8; // last 8 bits of APPS2_voltage
    buf0[4] = BPPS_scaled_voltage & 0b11111111; // first 8 bits of BPPS_voltage
    buf0[5] = BPPS_scaled_voltage >> 8; // last 8 bits of BPPS_voltage
    buf0[6] = static_cast<uint8_t>(ETC_state.APPS_position_avg * 100);
    buf0[7] = static_cast<uint8_t>(ETC_state.BPPS_position * 100);

    uint8_t buf1[8];
    uint16_t front_BSE_scaled_voltage = static_cast<uint16_t>(ETC_state.front_BSE_voltage * 1000);
    uint16_t rear_BSE_scaled_voltage = static_cast<uint16_t>(ETC_state.rear_BSE_voltage * 1000);
    buf1[0] = front_BSE_scaled_voltage & 0b11111111; // first 8 bits of front_BSE_voltage 
    buf1[1] = front_BSE_scaled_voltage >> 8; // last 8 bits of front_BSE_voltage 
    buf1[2] = rear_BSE_scaled_voltage & 0b11111111; // first 8 bits of rear_BSE_voltage 
    buf1[3] = rear_BSE_scaled_voltage >> 8; // last 8 bits of rear_BSE_voltage 
    buf1[4] = ETC_state.RTD_state |
        (ETC_state.motor_enabled << 1) |
        (ETC_state.APPS_deviation_implaus << 2) |
        (ETC_state.APPS_range_implaus << 3) |
        (ETC_state.BPPS_range_implaus << 4) |
        (ETC_state.brake_and_accel_implaus << 5) |
        (ETC_state.can_regen << 6) |
        (ETC_state.must_use_hydraulic_brakes << 7);
    buf1[5] = ETC_state.TS_active;

    CANMessage msg0{393, buf0, 8};
    CANMessage msg1{394, buf1, 6};
    can.write(msg0);
    can.write(msg1);

    VCU_can_timeout.attach(&send_CAN_message, 1000ms / VCU_can_frequency);
}
