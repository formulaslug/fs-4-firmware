#ifndef VECTORNAV_IMU_H
#define VECTORNAV_IMU_H

#include "mbed.h"
#include <vectornav/HAL/Mutex.hpp>
#include <vectornav/HAL/Thread.hpp>
#include <vectornav/Interface/Sensor.hpp>

struct VectornavState {
    /// Acceleration in body-frame. [ms^-2]
    VN::Vec3f accel{0,0,0};

    /// Angular rate in body-frame (Gyro) [rad/s]
    VN::Vec3f ang_rate{0,0,0};

    /// Yaw Pitch Roll 3-2-1 Euler angles with respect to NED. [deg]
    VN::Ypr ypr{0,0,0};

    /// Position: Latitude [deg], Longitude [deg], Altitude []
    VN::Lla pos{0,0,0};

    /// Velocity in Body-Frame [m/s]
    VN::Vec3f vel{0,0,0};
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
    */
    VN::Error start();

    /** Disconnects from the IMU
     *
 accel  */
    void disconnect();

    /** Gets the latest data from the sensor AND puts it in the state, run once per cycle or whenever you need data
     *
    */
    void update_state(VectornavState &state);

    private:
    // Pins
    const PinName tx; // TX pin
    const PinName rx; // RX pin

    // Sensor and latest data
    VN::Sensor sensor{}; // IMU Sensor object
    VN::Sensor::CompositeDataQueueReturn composite_data;

    VN::Registers::System::BinaryOutput1 imu_reg;
    VN::Registers::System::BinaryOutput2 ins_reg;
    VN::Registers::System::BinaryOutput3 attitude_reg;

    // Wraps Yaw Pitch Roll to [-180, 180)
    inline VN::Ypr wrap_ypr(VN::Ypr ypr) {
        ypr.yaw = wrap_angle(ypr.yaw);
        ypr.pitch = wrap_angle(ypr.pitch);
        ypr.roll = wrap_angle(ypr.roll);
        return ypr;
    }

    // Wraps angle to [-180, 180)
    inline float wrap_angle(float angle) {
        if (angle >= 180.0) {
            return angle - 360.0;
        } else if (angle < -180.0) {
            return  angle + 360.0;
        }
        return angle;
    }

    // Macro for checking/tracing errors
    inline void check_vn_error(VN::Error err) {
    if (err != VN::Error::None) {
        printf("VN: Error %hu encountered at %s:%d!\n", static_cast<uint16_t>(err), __FILE_NAME__, __LINE__);
    } else {
        printf("Binary output messages configured.\n");
    }
}
};

#endif
