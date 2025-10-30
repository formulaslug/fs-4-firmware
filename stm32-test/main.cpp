#include "mbed.h"

CAN can{PA_11, PA_12, 500000};

int main() {
    printf("Hello World!!\n");

    const uint8_t buf[] = {0xDE, 0xAD, 0xBE, 0xEF};
    CANMessage msg{0x04, buf, 4};

    while (true) {
        can.write(msg);
        printf("Sent message!\n");
        ThisThread::sleep_for(50ms);
    }

    return 0;
}
