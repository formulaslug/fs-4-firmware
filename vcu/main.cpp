#include "mbed.h"
#include "etc/etc_controller.h"

CAN can{PA_11, PA_12, 500000}; // placeholder baud rate
static constexpr uint32_t MOTOR_COMMAND_CAN_ID = 0x05; // placeholder ID

int main() {
    printf("Running\n");

    ETCController etc;
    etc.setTSActive(true);

    CANMessage rx;
    while (true) {
        if (can.read(rx)) {
            printf("CAN RX id=%u len=%u\n", rx.id, rx.len); // TODO: process inbound CAN commands when the bus is connected.
        }

        ETCState state = etc.sample();
        etc.updateRTD();
        state.rtdEnabled = etc.isRTDEnabled();
        state.torqueAllowed = etc.isTorqueAllowed();

        
        uint8_t payload[8] = {0};
        payload[0] = state.torqueAllowed ? 1 : 0;
        payload[1] = static_cast<uint8_t>(state.pedalTravel * 255.0f);
        payload[2] = static_cast<uint8_t>(state.travel1 * 255.0f);
        payload[3] = static_cast<uint8_t>(state.travel2 * 255.0f);
        payload[4] = state.implausLatched ? 1 : 0;
        payload[5] = state.rtdEnabled ? 1 : 0;
        payload[6] = state.brakePressed ? 1 : 0;

        CANMessage motorMessage(MOTOR_COMMAND_CAN_ID, payload, sizeof(payload));
        can.write(motorMessage);

        printf("ETC state: pedal=%.2f torque=%d implaus=%d rtd=%d\n",
               state.pedalTravel,
               state.torqueAllowed,
               state.implausLatched,
               state.rtdEnabled);

        ThisThread::sleep_for(50ms);
    }

    return 0;
}
