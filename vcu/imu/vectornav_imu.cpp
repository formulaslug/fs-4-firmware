#include "vectornav_imu.h"

VectorNavIMU::VectorNavIMU(PinName tx, PinName rx)
    : tx(tx), rx(rx) {}

VN::Error VectorNavIMU::connect() {
    // First try connecting at 921600, which it should be at
    // If it doesn't work, connect at 115200, which is default after a reset
    // Try 5 times with 100ms delay
    VN::Error err;
    VN::Registers::System::Model modelRegister;
    for (int i = 0; i < 5; i++) {
        err = sensor.connect(
            tx, rx, VN::Registers::System::BaudRate::BaudRates::Baud921600
        );

        if (this->sensor.readRegister(&modelRegister) == VN::Error::None) {
            return VN::Error::None;
        }

        ThisThread::sleep_for(100ms);
    }

    err = this->sensor.readRegister(&modelRegister);
    if (err != VN::Error::None) {
        sensor.disconnect();
        err = sensor.connect(
            tx, rx, VN::Registers::System::BaudRate::BaudRates::Baud115200
        );

        if (err != VN::Error::None) {
            return err;
        }

        err = sensor.changeBaudRate(
            VN::Registers::System::BaudRate::BaudRates::Baud921600, VN::Registers::System::BaudRate::SerialPort::Serial2
        );
        return err;
    }

    return err;
}

void VectorNavIMU::disconnect() {
    sensor.disconnect();
}

VN::Error VectorNavIMU::init() {
    // init registers
    raw_imu_reg.asyncMode.emplace();
    raw_imu_reg.asyncMode->serial1 = false;
    raw_imu_reg.asyncMode->serial2 = true;
    raw_imu_reg.rateDivisor = 1;

    nav_reg.asyncMode.emplace();
    nav_reg.asyncMode->serial1 = false;
    nav_reg.asyncMode->serial2 = true;
    nav_reg.rateDivisor = 8;

    time_reg.asyncMode.emplace();
    time_reg.asyncMode->serial1 = false;
    time_reg.asyncMode->serial2 = true;
    time_reg.rateDivisor = 80;

    // set values of what data we want
    raw_imu_reg.imu.accel = true;
    raw_imu_reg.imu.angularRate = true;
    raw_imu_reg.imu.mag = true;
    raw_imu_reg.imu.temperature = true;

    nav_reg.attitude.ypr = true;
    nav_reg.attitude.quaternion = true;
    nav_reg.ins.posLla = true;
    nav_reg.ins.velBody = true;
    nav_reg.ins.velNed = true;

    VN::Error err = sensor.writeRegister(&raw_imu_reg);
    if (err != VN::Error::None) return err;

    err = sensor.writeRegister(&nav_reg);
    if (err != VN::Error::None) return err;

    err = sensor.writeRegister(&time_reg);
    return err;
}


void VectorNavIMU::refreshDataToState(VectornavState &state) {
    std::optional<VN::CompositeData> rawImuData, navData, timeData;
    while (true) {
        VN::Sensor::CompositeDataQueueReturn compositeData =
            sensor.getNextMeasurement(false);

        if (!compositeData) {
            break;
        }

        if (compositeData->matchesMessage(raw_imu_reg)) {
            rawImuData = *compositeData;
        } else if (compositeData->matchesMessage(nav_reg)) {
            navData = *compositeData;
        } else if (compositeData->matchesMessage(time_reg)) {
            timeData = *compositeData;
        }
    }

    if (rawImuData->imu.accel.has_value()) {
        state.accel = rawImuData->imu.accel.value();
    }
    if (rawImuData->imu.angularRate.has_value()) {
        state.angRate = rawImuData->imu.angularRate.value();
    }
    if (rawImuData->imu.mag.has_value()) {
        state.mag = rawImuData->imu.mag.value();
    }
    if (navData->attitude.ypr.has_value()) {
        state.ypr = navData->attitude.ypr.value();
    }
    if (navData->ins.posLla.has_value()) {
        state.pos = navData->ins.posLla.value();
    }
    if (navData->ins.velBody.has_value()) {
        state.velBody = navData->ins.velBody.value();
    }
}

std::optional<VN::AsyncError> VectorNavIMU::getAsyncError() {
    std::optional<VN::AsyncError> asyncError = sensor.getNextAsyncError();
    return asyncError;
}

const char* VectorNavIMU::getModel() {
    VN::Registers::System::Model modelRegister;
    VN::Error err = this->sensor.readRegister(&modelRegister);
    CHECK_VN_ERR(err);

    const char* modelNumber = modelRegister.model.c_str();
    return modelNumber;
}

uint32_t VectorNavIMU::getConnectedBaudRate() {
    auto baud = this->sensor.connectedBaudRate();
    return static_cast<uint32_t>(*baud);
}