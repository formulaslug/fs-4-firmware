#ifndef VECTORNAV_IMU_H
#define VECTORNAV_IMU_H

#include "mbed.h"
#include <vectornav/HAL/Mutex.hpp>
#include <vectornav/HAL/Thread.hpp>
#include <vectornav/Interface/Sensor.hpp>

// Macro for checking/tracing errors
inline void check_vn_error(const char* file, int line, VN::Error err) {
    if (err != VN::Error::None) {
        printf("VN: Error %hu encountered at %s:%d!\n", static_cast<uint16_t>(err), file, line);
    }
}

#define CHECK_VN_ERR(err) check_vn_error(__FILE__, __LINE__, err)

class VectorNavIMU {
    public:
    VectorNavIMU(PinName tx, PinName rx);
    VN::Error connect();
    void disconnect();

    // Register Management
    void initRegisters();

    void enableRawImu(bool enabled, uint16_t rateDivisor = 1);
    void enableAttitude(bool enabled, uint16_t rateDivisor = 8);
    void enableNavigation(bool enabled, uint16_t rateDivisor = 8);
    void enableGpsTime(bool enabled, uint16_t rateDivisor = 80);
    void enableUncertainty(bool enabled);
    void enableStatus(bool enabled);

    void disableAll();

    VN::Error applyRegisters();

    void refreshData();

    // Getters
    std::optional<VN::Vec3f> getAccel() const;
    std::optional<VN::Vec3f> getAngularRate() const;
    std::optional<VN::Vec3f> getMag() const;
    std::optional<VN::Ypr> getYawPitchRoll() const;
    std::optional<VN::Lla> getPosition() const;
    std::optional<VN::Vec3f> getVelocityBody() const;


    // Util
    std::optional<VN::AsyncError> getAsyncError();
    const char* getModel();
    uint32_t getConnectedBaudRate();

    private:
    // Connection Config
    const PinName tx;
    const PinName rx;

    // Sensor and latest data
    VN::Sensor sensor{};
    std::optional<VN::CompositeData> lastRawImuData;
    std::optional<VN::CompositeData> lastNavData;
    std::optional<VN::CompositeData> lastTimeData;

    // Registers
    VN::Registers::System::BinaryOutput1 raw_imu_reg;
    VN::Registers::System::BinaryOutput2 nav_reg;
    VN::Registers::System::BinaryOutput3 time_reg;
};

#endif