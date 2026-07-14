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
    imu_reg.asyncMode->serial2 = true; // should be true
    imu_reg.rateDivisor = 8; // 800Hz / 8 = 100Hz
    imu_reg.imu.accel = true;
    imu_reg.imu.angularRate = true;

    ins_reg.asyncMode.emplace();
    ins_reg.asyncMode->serial1 = false;
    ins_reg.asyncMode->serial2 = true;
    ins_reg.rateDivisor = 8; // 800Hz / 8 = 100Hz
    ins_reg.ins.posLla = true;
    ins_reg.ins.velBody = true;

    attitude_reg.asyncMode.emplace();
    attitude_reg.asyncMode->serial1 = false;
    attitude_reg.asyncMode->serial2 = true;
    attitude_reg.rateDivisor = 8; // 800Hz / 8 = 100Hz
    attitude_reg.attitude.ypr = true;

    err = sensor.writeRegister(&imu_reg);
    check_vn_error(err);
    err = sensor.writeRegister(&ins_reg);
    check_vn_error(err);
    err = sensor.writeRegister(&attitude_reg);
    check_vn_error(err);

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
        state.accel = composite_data->imu.accel.has_value() ? composite_data->imu.accel.value() : state.accel;
        state.ang_rate = composite_data->imu.angularRate.has_value() ? composite_data->imu.angularRate.value() : state.ang_rate;
    }
    if (composite_data->matchesMessage(ins_reg)) {
        state.pos = composite_data->ins.posLla.has_value() ? composite_data->ins.posLla.value() : state.pos;
        state.vel = composite_data->ins.velBody.has_value() ? composite_data->ins.velBody.value() : state.vel;
    }
    if (composite_data->matchesMessage(attitude_reg)) {
        state.ypr = composite_data->attitude.ypr.has_value() ? composite_data->attitude.ypr.value() : state.ypr;
    }

    // Handle asynchronous errors
    std::optional<VN::AsyncError> asyncError = sensor.getNextAsyncError();
    if (asyncError.has_value()) {
        printf("Received async error: %s\n", asyncError.value().message.data());
    }
}
