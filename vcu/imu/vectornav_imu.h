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
    VN::Error connect();

    /** Disconnects from the IMU
     *
    */
    void disconnect();

    /** Sensor initialization sequence:
     * 1. Call initRegisters() before enabling any functions
     * 2. Enable selected functions by calling their enable method
     * 3. Call applyRegisters() to send config to the IMU
    */

    /** Initializes Default Register Values
     *
    */
    void initRegisters();

    /** Enables raw IMU functionality (accel, angular rate, magnetometer, temp)
     *
     * @param enabled Whether to enable the function
     * @param rateDivisor The divisor for sampling rate. i.e. if sensor running at 800MHz, and rateDivisor is 8, 800/1=800MHz
    */
    void enableRawImu(bool enabled, uint16_t rateDivisor = 1);

    /** Enables attitude (YPR, Quaternion)
     *
     * @param enabled Whether to enable the function
     * @param rateDivisor The divisor for sampling rate. i.e. if sensor running at 800MHz, and rateDivisor is 8, 800/8=100MHz
    */
    void enableAttitude(bool enabled, uint16_t rateDivisor = 8);

    /** Enables navigation (position, velocity body, velocity ned)
     *
     * @param enabled Whether to enable the function
     * @param rateDivisor The divisor for sampling rate. i.e. if sensor running at 800MHz, and rateDivisor is 8, 800/8=100MHz
    */
    void enableNavigation(bool enabled, uint16_t rateDivisor = 8);

    /** Enables GPS time (GPS time, UTC, Week, Time of Week)
     *
     * @param enabled Whether to enable the function
     * @param rateDivisor The divisor for sampling rate. i.e. if sensor running at 800MHz, and rateDivisor is 80, 800/80=10MHz
    */
    void enableGpsTime(bool enabled, uint16_t rateDivisor = 80);

    /** Enables uncertainty (position uncertainty, velocity uncertainty, ypr uncertainty)
     *
     * @param enabled Whether to enable the function
    */
    void enableUncertainty(bool enabled);

    /** Enables status (imu status, ins status)
     *
     * @param enabled Whether to enable the function
    */
    void enableStatus(bool enabled);

    /** Disables all functions
     *
    */
    void disableAll();

    /** Sends all register configurations to the sensor
     *
     * @returns VN::Error::None if succeeded, otherwise an error under VN::Error
    */
    VN::Error applyRegisters();

    /** Gets the latest data from the sensor, run once per cycle or whenever you need data
     *
    */
    void refreshData();

    /** Get acceleration
     *
     * @returns a vector of accelerations, wrapped in an optional
    */
    std::optional<VN::Vec3f> getAccel() const;

    /** Get angular rate
     *
     * @returns a vector of angular rates, wrapped in an optional
    */
    std::optional<VN::Vec3f> getAngularRate() const;

    /** Get magnetometer data
     *
     * @returns a vector of magnetometer values, wrapped in an optional
    */
    std::optional<VN::Vec3f> getMag() const;

    /** Get yaw, pitch, and roll
     *
     * @returns a data type with .yaw, .pitch, and .roll, wrapped in an optional
    */
    std::optional<VN::Ypr> getYawPitchRoll() const;
    
    /** Get position
     *
     * @returns a data type with .lat, .lon, .alt,  wrapped in an optional
    */
    std::optional<VN::Lla> getPosition() const;

    /** Get body velocity
     *
     * @returns a vector of body velocities, wrapped in an optional
    */
    std::optional<VN::Vec3f> getVelocityBody() const;


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
    std::optional<VN::CompositeData> lastRawImuData; // Latest fetched raw data
    std::optional<VN::CompositeData> lastNavData; // Latest fetched nav data
    std::optional<VN::CompositeData> lastTimeData; // Latest fetched time data

    // Registers
    VN::Registers::System::BinaryOutput1 raw_imu_reg; // Raw IMU register
    VN::Registers::System::BinaryOutput2 nav_reg; // Nav register
    VN::Registers::System::BinaryOutput3 time_reg; // Time register
};

#endif