#ifndef VECTORNAV_IMU_H
#define VECTORNAV_IMU_H

#include "mbed.h"
#include <vectornav/HAL/Mutex.hpp>
#include <vectornav/HAL/Thread.hpp>
#include <vectornav/Interface/Sensor.hpp>

inline void check_vn_error(const char* file, int line, VN::Error err) {
    if (err != VN::Error::None) {
        printf("VN: Error %hu encountered at %s:%d!\n", static_cast<uint16_t>(err), file, line);
    }
}

#define CHECK_VN_ERR(err) check_vn_error(__FILE__, __LINE__, err)

class VectorNavIMU {
    public:
    VectorNavIMU(PinName tx, PinName rx, VN::Registers::System::BaudRate::BaudRates baudRate);
    VN::Error connect();
    VN::Error setRegisters();
    VN::Vec3f getData();
    std::optional<VN::AsyncError> getAsyncError();
    const char* getModel();
    uint32_t getConnectedBaudRate();
    void disconnect();

    private:
    VN::Registers::System::BaudRate::BaudRates baudRate;
    const PinName tx;
    const PinName rx;
    VN::Sensor sensor{};
    VN::Sensor::CompositeDataQueueReturn compositeData;

    VN::Registers::System::BinaryOutput1 binary_out_1_reg;
    VN::Registers::System::BinaryOutput2 binary_out_2_reg;
    VN::Registers::System::BinaryOutput3 binary_out_3_reg;

};

#endif