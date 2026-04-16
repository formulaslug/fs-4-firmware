#include "mbed.h"
#include <vectornav/HAL/Mutex.hpp>
#include <vectornav/HAL/Thread.hpp>
#include <vectornav/Interface/Sensor.hpp>

// CAN can{PA_11, PA_12, 500000};

VN::Sensor sensor{};

inline void check_vn_error(VN::Error err) {
    if (err != VN::Error::None) {
        printf("VN: Error %hu encountered at %s:%d!\n", static_cast<uint16_t>(err), __FILE__, __LINE__);
    } else {
        printf("Binary output messages configured.\n");
    }
}

int main() {
    printf("Hello World!\n");

    // const uint8_t buf[] = {0xDE, 0xAD, 0xBE, 0xEF};
    // CANMessage msg{0x04, buf, 4};
    // while (true) {
    //     can.write(msg);
    //     printf("Sent message!\n");
    //
    //     ThisThread::sleep_for(50ms);
    // }

    VN::Error err = sensor.connect(PC_12, PD_2, VN::Registers::System::BaudRate::BaudRates::Baud115200);
    if (err != VN::Error::None) {
        printf("Error connecting to sensor! (%hu)\n", static_cast<uint16_t>(err));
    }
    printf("Connected to sensor!\n");

    // Poll and print the model number using a read register command
    // Create an empty register object of the necessary type, where the data member will be populated when the sensor responds to our "read register" request
    VN::Registers::System::Model modelRegister;
    err = sensor.readRegister(&modelRegister);
    check_vn_error(err);

    const char* modelNumber = modelRegister.model.c_str();
    printf("Sensor Model Number: %s\n", modelNumber);

    auto baud = sensor.connectedBaudRate();
    printf("baud: %d\n", static_cast<uint32_t>(*baud));

    sensor.changeBaudRate(
        VN::Registers::System::BaudRate::BaudRates::Baud921600,
        VN::Registers::System::BaudRate::SerialPort::Serial2
    );
    check_vn_error(err);

    baud = sensor.connectedBaudRate();
    printf("baud: %d\n", static_cast<uint32_t>(*baud));

    VN::Registers::System::BinaryOutput1 binary_out_1_reg;
    VN::Registers::System::BinaryOutput2 binary_out_2_reg;
    VN::Registers::System::BinaryOutput3 binary_out_3_reg;

    binary_out_1_reg.asyncMode.emplace();
    binary_out_1_reg.asyncMode->serial1 = false;
    binary_out_1_reg.asyncMode->serial2 = true;
    binary_out_1_reg.rateDivisor = 1; // 800Hz / 1 = 800Hz
    binary_out_1_reg.imu.accel = true;

    binary_out_2_reg.asyncMode.emplace();
    binary_out_2_reg.asyncMode->serial1 = false;
    binary_out_2_reg.asyncMode->serial2 = true;
    binary_out_2_reg.rateDivisor = 8; // 800Hz / 8 = 100Hz
    binary_out_2_reg.ins.posLla = true;
    binary_out_2_reg.ins.posU = true;
    binary_out_2_reg.ins.velBody = true;
    binary_out_2_reg.ins.velU = true;
    binary_out_2_reg.attitude.ypr = true;
    binary_out_2_reg.attitude.yprU = true;

    binary_out_3_reg.asyncMode.emplace();
    binary_out_3_reg.asyncMode->serial1 = false;
    binary_out_3_reg.asyncMode->serial2 = true;
    binary_out_3_reg.rateDivisor = 1; // 800Hz / 80 = 10Hz
    binary_out_3_reg.time.timeGps = true;

    // err = sensor.writeRegister(&binary_out_1_reg);
    err = sensor.writeRegister(&binary_out_2_reg);
    // err = sensor.writeRegister(&binary_out_3_reg);
    check_vn_error(err);

    VN::Sensor::CompositeDataQueueReturn compositeData = sensor.getMostRecentMeasurement();
    while (1) {
        compositeData = sensor.getNextMeasurement();
        // Check to make sure that a measurement is available
        if (!compositeData) continue;

        if (compositeData->matchesMessage(binary_out_1_reg)) {
            printf("\x1b[2J");
            printf("\x1b[H");

            printf("Found binary 1 measurment.\n");

            VN::Vec3f accel = compositeData->imu.accel.value();
            printf("\tAccel X: %f\n\tAccel Y: %f\n\t Accel Z: %f\n", accel[0], accel[1], accel[2]);
        }

        // Handle asynchronous errors
        std::optional<VN::AsyncError> asyncError = sensor.getNextAsyncError();
        if (asyncError.has_value()) {
            printf("Received async error: %s\n", asyncError.value().message.data());
        }
    }

    sensor.disconnect();

    while (1) {};
}
