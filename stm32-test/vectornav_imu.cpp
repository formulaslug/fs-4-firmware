#include "vectornav_imu.h"

VectorNavIMU::VectorNavIMU(PinName tx, PinName rx, VN::Registers::System::BaudRate::BaudRates baudRate)
    : baudRate(baudRate), tx(tx), rx(rx) {}

VN::Error VectorNavIMU::connect() {
    VN::Error err = sensor.connect(
        tx, rx, VN::Registers::System::BaudRate::BaudRates::Baud115200
    );
    if (err != VN::Error::None) {
        return err;
    }

    if (baudRate != VN::Registers::System::BaudRate::BaudRates::Baud115200) {
        err = sensor.changeBaudRate(
            baudRate, VN::Registers::System::BaudRate::SerialPort::Serial2
        );
    }
    return err;
}

void VectorNavIMU::disconnect() {
    sensor.disconnect();
}

void VectorNavIMU::initRegisters() {
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
}

void VectorNavIMU::enableRawImu(bool enabled, uint16_t rateDivisor) {
    raw_imu_reg.rateDivisor = rateDivisor;

    raw_imu_reg.imu.accel = enabled;
    raw_imu_reg.imu.angularRate = enabled;
    raw_imu_reg.imu.mag = enabled;
    raw_imu_reg.imu.temperature = enabled;
}

void VectorNavIMU::enableAttitude(bool enabled, uint16_t rateDivisor) {
    nav_reg.rateDivisor = rateDivisor;

    nav_reg.attitude.ypr = enabled;
    nav_reg.attitude.quaternion = enabled;
}

void VectorNavIMU::enableNavigation(bool enabled, uint16_t rateDivisor) {
    nav_reg.rateDivisor = rateDivisor;

    nav_reg.ins.posLla = enabled;
    nav_reg.ins.velBody = enabled;
    nav_reg.ins.velNed = enabled;
}

void VectorNavIMU::enableUncertainty(bool enabled) {
    nav_reg.ins.posU = enabled;
    nav_reg.ins.velU = enabled;
    nav_reg.attitude.yprU = enabled;
}

void VectorNavIMU::enableStatus(bool enabled) {
    raw_imu_reg.imu.imuStatus = enabled;
    nav_reg.ins.insStatus = enabled;
    nav_reg.common.insStatus = enabled;
}

void VectorNavIMU::enableGpsTime(bool enabled, uint16_t rateDivisor) {
    time_reg.rateDivisor = rateDivisor;

    time_reg.time.timeGps = enabled;
    time_reg.time.timeGpsTow = enabled;
    time_reg.time.timeGpsWeek = enabled;
    time_reg.time.timeUtc = enabled;
}

void VectorNavIMU::disableAll() {
    raw_imu_reg.common = 0;
    raw_imu_reg.time = 0;
    raw_imu_reg.imu = 0;
    raw_imu_reg.gnss = 0;
    raw_imu_reg.attitude = 0;
    raw_imu_reg.ins = 0;

    nav_reg.common = 0;
    nav_reg.time = 0;
    nav_reg.imu = 0;
    nav_reg.gnss = 0;
    nav_reg.attitude = 0;
    nav_reg.ins = 0;

    time_reg.common = 0;
    time_reg.time = 0;
    time_reg.imu = 0;
    time_reg.gnss = 0;
    time_reg.attitude = 0;
    time_reg.ins = 0;
}

VN::Error VectorNavIMU::applyRegisters() {
    VN::Error err = sensor.writeRegister(&raw_imu_reg);
    if (err != VN::Error::None) return err;

    err = sensor.writeRegister(&nav_reg);
    if (err != VN::Error::None) return err;

    err = sensor.writeRegister(&time_reg);
    return err;
}

void VectorNavIMU::refreshData() {
    while (true) {
        VN::Sensor::CompositeDataQueueReturn compositeData =
            sensor.getNextMeasurement(false);

        if (!compositeData) {
            break;
        }

        if (compositeData->matchesMessage(raw_imu_reg)) {
            lastRawImuData = *compositeData;
        } else if (compositeData->matchesMessage(nav_reg)) {
            lastNavData = *compositeData;
        } else if (compositeData->matchesMessage(time_reg)) {
            lastTimeData = *compositeData;
        }
    }
}

std::optional<VN::Vec3f> VectorNavIMU::getAccel() const {
    if (!lastRawImuData || !lastRawImuData->imu.accel.has_value()) {
        return std::nullopt;
    }

    return lastRawImuData->imu.accel.value();
}

std::optional<VN::Vec3f> VectorNavIMU::getAngularRate() const {
    if (!lastRawImuData || !lastRawImuData->imu.angularRate.has_value()) {
        return std::nullopt;
    }

    return lastRawImuData->imu.angularRate.value();
}

std::optional<VN::Vec3f> VectorNavIMU::getMag() const {
    if (!lastRawImuData || !lastRawImuData->imu.mag.has_value()) {
        return std::nullopt;
    }

    return lastRawImuData->imu.mag.value();
}

std::optional<VN::Ypr> VectorNavIMU::getYawPitchRoll() const {
    if (!lastNavData || !lastNavData->attitude.ypr.has_value()) {
        return std::nullopt;
    }

    return lastNavData->attitude.ypr.value();
}

std::optional<VN::Lla> VectorNavIMU::getPosition() const {
    if (!lastNavData || !lastNavData->ins.posLla.has_value()) {
        return std::nullopt;
    }

    return lastNavData->ins.posLla.value();
}

std::optional<VN::Vec3f> VectorNavIMU::getVelocityBody() const {
    if (!lastNavData || !lastNavData->ins.velBody.has_value()) {
        return std::nullopt;
    }

    return lastNavData->ins.velBody.value();
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