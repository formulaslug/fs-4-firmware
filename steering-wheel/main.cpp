#include "BT817Q.hpp"
#include "dash_screen.hpp"
#include "mbed.h"

DashScreen screen{
    PA_1, PA_1, PA_1, PA_1, PA_1, PA_1, EvePresets::CFA800480E3
};

CAN can{PA_11, PA_12, 500000};

int main() {
    printf("Hello World!!\n");

    // Todo - read CAN, decode relevant data, render screen

    CANMessage msg;
    while (true) {

        ThisThread::sleep_for(50ms);
    }

    return 0;
}
