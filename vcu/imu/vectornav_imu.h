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

struct VectornavState {
    /// Acceleration in body-frame. [ms^-2]
    VN::Vec3f accelBody{0,0,0};

    /// Angular rate in body-frame (Gyro) [rad/s]
    VN::Vec3f angRate{0,0,0};

    /// Yaw Pitch Roll 3-2-1 Euler angles with respect to NED. [deg]
    VN::Ypr ypr{0,0,0};

    /// Position: Latitude [deg], Longitude [deg], Altitude []
    VN::Lla pos{0,0,0};

    /// Velocity in Body-Frame [m/s]
    VN::Vec3f velBody{0,0,0};
};

class VectorNavIMU {
    public:
    /** Creates a VectorNavIMU
     *
     * @param tx UART TX pin
     * @param rx UART RX pin
    */
    VectorNavIMU(PinName tx, PinName rx);

    /** Connects to the IMU
     *
     * @returns VN::Error::None if succeeded, otherwise one of the errors under VN::Error
     * Important: spawns a thread
    */
    VN::Error connect();

    /** Disconnects from the IMU
     *
    */
    void disconnect();

    /** Enables raw IMU functionality (accel, angular rate, magnetometer, temp)
     * Initializes the sensor and writes registers to it
     * @returns VN::Error::None if succeeded, otherwise an error under VN::Error
    */
    VN::Error init();

    /** Gets the latest data from the sensor AND puts it in the state, run once per cycle or whenever you need data
     *
    */
    void refreshDataToState(VectornavState &state);


    /** Get the next async error, if any happened
     *
     * @returns an AsyncError wrapped in an optional
    */
    std::optional<VN::AsyncError> getAsyncError();

    /** Get the model name of the sensor
     *
     * @returns the model name
    */
    const char* getModel();

    /** Get the connected baud rate
     *
     * @returns the connector baud rate
    */
    uint32_t getConnectedBaudRate();

    private:
    // Pins
    const PinName tx; // TX pin
    const PinName rx; // RX pin

    // Sensor and latest data
    VN::Sensor sensor{}; // IMU Sensor object

    // Registers
    VN::Registers::System::BinaryOutput1 raw_imu_reg; // Raw IMU register
    VN::Registers::System::BinaryOutput2 nav_reg; // Nav register
    VN::Registers::System::BinaryOutput3 time_reg; // Time register

    /// Wraps Yaw Pitch Roll to [-180, 180)
    inline VN::Ypr wrap_ypr(VN::Ypr ypr) {
        ypr.yaw = wrap_angle(ypr.yaw);
        ypr.pitch = wrap_angle(ypr.pitch);
        ypr.roll = wrap_angle(ypr.roll);
        return ypr;
    }

    /// Wraps angle to [-180, 180)
    inline float wrap_angle(float angle) {
        if (angle >= 180.0) {
            return angle - 360.0;
        } else if (angle < -180.0) {
            return  angle + 360.0;
        }
    }
};

#endif
