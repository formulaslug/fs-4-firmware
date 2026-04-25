#include "mbed.h"
#include "vectornav_imu.h"

// CAN can{PA_11, PA_12, 500000};

VectorNavIMU imu(PC_12, PD_2, VN::Registers::System::BaudRate::BaudRates::Baud921600);

int main() {
    printf("Hello World!\n");

    CHECK_VN_ERR(imu.connect());
    printf("Connected to sensor!\n");

    printf("Sensor Model Number: %s\n", imu.getModel());

    printf("baud: %d\n", imu.getConnectedBaudRate());

    CHECK_VN_ERR(imu.setRegisters());

    while (1) {

        VN::Vec3f accel = imu.getData();
        printf("\tAccel X: %f\n\tAccel Y: %f\n\t Accel Z: %f\n", accel[0], accel[1], accel[2]);

        // Handle asynchronous errors
        std::optional<VN::AsyncError> asyncError = imu.getAsyncError();
        if (asyncError.has_value()) {
            printf("Received async error: %s\n", asyncError.value().message.data());
        }
    }

    imu.disconnect();

    while (1) {};
}
