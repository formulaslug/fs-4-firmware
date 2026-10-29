#ifndef VECTORNAV_IMU_H
#define VECTORNAV_IMU_H

#include "mbed.h"
#include <vectornav/HAL/Mutex.hpp>
#include <vectornav/HAL/Thread.hpp>
#include <vectornav/Interface/Sensor.hpp>

// Macro for checking/tracing errors
#define return_if_vn_error(vn_err)                                                                 \
    do {                                                                                           \
        if (vn_err != VN::Error::None) {                                                           \
            printf(                                                                                \
                "VN Error: %s (%s:%d)\n",                                                          \
                std::string(VN::errorCodeToString(vn_err)).c_str(),                                \
                __FILE_NAME__,                                                                     \
                __LINE__                                                                           \
            );                                                                                     \
            return vn_err;                                                                         \
        }                                                                                          \
    } while (0)

struct VectornavState {
    /// Acceleration in body-frame. [ms^-2]
    VN::Vec3f accel{0, 0, 0};

    /// Angular rate in body-frame (Gyro) [rad/s]
    VN::Vec3f ang_rate{0, 0, 0};

    /// Yaw Pitch Roll 3-2-1 Euler angles with respect to NED. [deg]
    VN::Ypr ypr{0, 0, 0};

    /// Position: Latitude [deg], Longitude [deg], Altitude []
    VN::Lla pos{0, 0, 0};

    /// Velocity in Body-Frame [m/s]
    VN::Vec3f vel{0, 0, 0};

    // Fixed: this was uninitialized, and it is the GNSS fix type (gnss1Fix), not a satellite count
    uint8_t gnss_fix = 0;
};

enum class VectornavLink : uint8_t {
    Disconnected = 0,
    Configuring = 1,
    NoData = 2,
    Streaming = 3,
};

class VectorNavIMU {
public:
    using BaudRates = VN::Registers::System::BaudRate::BaudRates;

    // Fixed: 115200 baud capped the output rates. The VN-200 powers up at 115200, so we connect at
    // that and switch the link to 921600 (not saved to the sensor, so it's redone after a power-cycle)
    static constexpr BaudRates DEFAULT_BAUD = BaudRates::Baud115200;
    static constexpr BaudRates FAST_BAUD = BaudRates::Baud921600;

    /** Creates a VectorNavIMU
     *
     * @param tx UART TX pin
     * @param rx UART RX pin
     * @param restore VN-200 RESTORE pin
     */
    VectorNavIMU(PinName tx, PinName rx, PinName restore);

    /** Connects to the IMU
     *
     * @returns VN::Error::None if succeeded, otherwise one of the errors under VN::Error
     */
    VN::Error start();

    /** Disconnects from the IMU
     */
    void disconnect();

    /** Gets the latest data from the sensor AND puts it in the state, run once per cycle or
     * whenever you need data
     *
     */
    VN::Error update_state(VectornavState& state);

    VectornavLink link() const { return link_state; }
    uint32_t baud() const;
    uint16_t configure_count() const { return configure_attempts; }
    uint16_t error_count() const { return errors; }
    VN::Error last_error() const { return latest_error; }

private:
    // Fixed: start() used to run once at boot so if the sensor wasn't answering yet the IMU
    // stayed dead until a reboot. added retry if not connected
    static constexpr auto RETRY_PERIOD = 1s;
    // Fixed: outputs aren't saved to the sensor's flash (after talking to Luca), so after it
    // power-cycles it goes quiet. If timeout, reconfigure IMU
    static constexpr auto DATA_TIMEOUT = 500ms;

    // Pins
    const PinName tx; // TX pin
    const PinName rx; // RX pin
    DigitalOut restore;

    // Sensor and latest data
    VN::Sensor sensor{}; // IMU Sensor object
    bool connected = false;
    bool configured = false;

    VN::Registers::System::BinaryOutput1 imu_reg;
    VN::Registers::System::BinaryOutput2 nav_reg;

    VectornavLink link_state = VectornavLink::Disconnected;
    Kernel::Clock::time_point last_configure{};
    Kernel::Clock::time_point last_packet{};
    uint16_t configure_attempts = 0;
    uint16_t errors = 0;
    VN::Error latest_error = VN::Error::None;

    void restore_factory_settings();
    VN::Error find_sensor();
    VN::Error configure();
    void record_error(VN::Error err);

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
            return angle + 360.0;
        }
        return angle;
    }
};

#endif
