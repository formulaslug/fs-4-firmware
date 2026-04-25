#include "vectornav_imu.h"

VectorNavIMU::VectorNavIMU(PinName tx, PinName rx, VN::Registers::System::BaudRate::BaudRates baudRate)
    : baudRate(baudRate), tx(tx), rx(rx) {}

VN::Error VectorNavIMU::connect() {
    VN::Error err = this->sensor.connect(
        this->tx, this->rx, VN::Registers::System::BaudRate::BaudRates::Baud115200
    );
    if (err != VN::Error::None) {
        return err;
    }

    if (this->baudRate != VN::Registers::System::BaudRate::BaudRates::Baud115200) {
        err = this->sensor.changeBaudRate(
            this->baudRate, VN::Registers::System::BaudRate::SerialPort::Serial2
        );
    }
    return err;
}

VN::Error VectorNavIMU::setRegisters() {
    this->binary_out_1_reg.asyncMode.emplace();
    this->binary_out_1_reg.asyncMode->serial1 = false;
    this->binary_out_1_reg.asyncMode->serial2 = true;
    this->binary_out_1_reg.rateDivisor = 1; // 800Hz / 1 = 800Hz
    this->binary_out_1_reg.imu.accel = true;

    this->binary_out_2_reg.asyncMode.emplace();
    this->binary_out_2_reg.asyncMode->serial1 = false;
    this->binary_out_2_reg.asyncMode->serial2 = true;
    this->binary_out_2_reg.rateDivisor = 8; // 800Hz / 8 = 100Hz
    this->binary_out_2_reg.ins.posLla = true;
    this->binary_out_2_reg.ins.posU = true;
    this->binary_out_2_reg.ins.velBody = true;
    this->binary_out_2_reg.ins.velU = true;
    this->binary_out_2_reg.attitude.ypr = true;
    this->binary_out_2_reg.attitude.yprU = true;

    this->binary_out_3_reg.asyncMode.emplace();
    this->binary_out_3_reg.asyncMode->serial1 = false;
    this->binary_out_3_reg.asyncMode->serial2 = true;
    this->binary_out_3_reg.rateDivisor = 1; // 800Hz / 80 = 10Hz
    this->binary_out_3_reg.time.timeGps = true;

    this->sensor.writeRegister(&this->binary_out_1_reg);
    this->sensor.writeRegister(&this->binary_out_2_reg);
VN::Error err = this->sensor.writeRegister(&this->binary_out_3_reg);
    return err;
}

VN::Vec3f VectorNavIMU::getData() {
    if (!this->compositeData) {
        this->compositeData = this->sensor.getMostRecentMeasurement();
    }

    while (true) {
        this->compositeData = sensor.getNextMeasurement();
        // Check to make sure that a measurement is available
        if (!compositeData) continue;

        if (compositeData->matchesMessage(this->binary_out_1_reg)) {
            VN::Vec3f accel = compositeData->imu.accel.value();
            return accel;
        }
    }
}

std::optional<VN::AsyncError> VectorNavIMU::getAsyncError() {
    std::optional<VN::AsyncError> asyncError = this->sensor.getNextAsyncError();
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

void VectorNavIMU::disconnect() {
    this->sensor.disconnect();
}