#include "vectornav_imu.h"
#include "vectornav/Implementation/MeasurementDatatypes.hpp"
#include "vectornav/Interface/Registers.hpp"
#include <cstdio>

VectorNavIMU::VectorNavIMU(PinName tx, PinName rx)
    : tx(tx), rx(rx) {}

VN::Error VectorNavIMU::start() {
    VN::Error err = sensor.connect(tx, rx, VN::Registers::System::BaudRate::BaudRates::Baud115200);
    if (err != VN::Error::None) {
        printf("Error connecting to sensor! (%hu)\n", static_cast<uint16_t>(err));
    }
    printf("Connected to sensor!\n");

    // Poll and print the model number using a read register command
    // Create an empty register object of the necessary type, where the data member will be populated when the sensor responds to our "read register" request
    VN::Registers::System::Model model_register;
    err = sensor.readRegister(&model_register);
    check_vn_error(err);

    const char* model_number = model_register.model.c_str();
    printf("Sensor Model Number: %s\n", model_number);

    auto baud = sensor.connectedBaudRate();
    printf("baud: %lu\n", static_cast<uint32_t>(*baud));

    // sensor.changeBaudRate(
    //     VN::Registers::System::BaudRate::BaudRates::Baud921600,
    //     VN::Registers::System::BaudRate::SerialPort::Serial2
    // );
    // check_vn_error(err);

    baud = sensor.connectedBaudRate();
    printf("baud: %lu\n", static_cast<uint32_t>(*baud));

    imu_reg.asyncMode.emplace();
    imu_reg.asyncMode->serial1 = false;
    imu_reg.asyncMode->serial2 = true;
    imu_reg.rateDivisor = 4; // 800Hz / 4 = 200Hz
    imu_reg.imu.accel = true;
    imu_reg.imu.angularRate = true;

    // ins_reg.asyncMode.emplace();
    // ins_reg.asyncMode->serial1 = false;
    // ins_reg.asyncMode->serial2 = true;
    // ins_reg.rateDivisor = 4; // 800Hz / 4 = 200Hz
    // ins_reg.ins.posLla = true;
    // // ins_reg.ins.posU = true;
    // ins_reg.ins.velBody = true;
    // // ins_reg.ins.velU = true;
    // // positioning_reg.attitude.ypr = true;
    // // positioning_reg.attitude.yprU = true;
    //
    // attitude_reg.asyncMode.emplace();
    // attitude_reg.asyncMode->serial1 = false;
    // attitude_reg.asyncMode->serial2 = true;
    // attitude_reg.rateDivisor = 4; // 800Hz / 80 = 10Hz
    // attitude_reg.attitude.ypr = true;
    // // gps_reg.attitude.yprU = true;
    // // gps_reg.time.timeGps = true;

    err = sensor.writeRegister(&imu_reg);
    check_vn_error(err);
    // err = sensor.writeRegister(&ins_reg);
    // check_vn_error(err);
    // err = sensor.writeRegister(&attitude_reg);
    // check_vn_error(err);

    composite_data = sensor.getMostRecentMeasurement();
    return err;
}

void VectorNavIMU::disconnect() {
    sensor.disconnect();
}

// if it doesn't update often enough, place IN A NEW THREAD and run in while(1) loop
void VectorNavIMU::update_state(VectornavState &state) {
    composite_data = sensor.getNextMeasurement();
    // Check to make sure that a measurement is available
    if (!composite_data) return;

    if (composite_data->matchesMessage(imu_reg)) {
        printf("\x1b[2J");
        printf("\x1b[H");

        printf("Found binary 1 measurment.\n");

        state.accel = composite_data->imu.accel.has_value() ? composite_data->imu.accel.value() : state.accel;
        state.ang_rate = composite_data->imu.angularRate.has_value() ? composite_data->imu.angularRate.value() : state.ang_rate;
        printf("\tAccel X: %f\n\tAccel Y: %f\n\tAccel Z: %f\n", state.accel[0], state.accel[1], state.accel[2]);
        printf("\tGyro X: %f\n\tGyro Y: %f\n\tGyro Z: %f\n", state.ang_rate[0], state.ang_rate[1], state.ang_rate[2]);
    }
    // if (composite_data->matchesMessage(ins_reg)) {
    //     printf("\x1b[2J");
    //     printf("\x1b[H");
    //
    //     printf("Found binary 1 measurment.\n");
    //
    //     state.pos = composite_data->ins.posLla.has_value() ? composite_data->ins.posLla.value() : state.pos;
    //     state.vel = composite_data->ins.velBody.has_value() ? composite_data->ins.velBody.value() : state.vel;
    //     printf("\tVelocity X: %f\n\tVelocity Y: %f\n\tVelocity Z: %f\n", state.vel[0], state.vel[1], state.vel[2]);
    // }
    // if (composite_data->matchesMessage(attitude_reg)) {
    //     printf("\x1b[2J");
    //     printf("\x1b[H");
    //
    //     printf("Found binary 1 measurment.\n");
    //
    //     state.ypr = composite_data->attitude.ypr.has_value() ? wrap_ypr(composite_data->attitude.ypr.value()) : state.ypr;
    //     printf("\tYaw: %f\n\tPitch: %f\n\tRoll: %f\n", state.ypr.yaw, state.ypr.pitch, state.ypr.roll);
    // }

    // Handle asynchronous errors
    std::optional<VN::AsyncError> asyncError = sensor.getNextAsyncError();
    if (asyncError.has_value()) {
        printf("Received async error: %s\n", asyncError.value().message.data());
    }
}
