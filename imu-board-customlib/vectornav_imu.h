#pragma once

#include "mbed.h"
#include <cstddef>
#include <cstdint>

struct VectornavState {
    /// Acceleration in body-frame. [ms^-2]
    float accel[3] = {0, 0, 0};

    /// Angular rate in body-frame (Gyro) [rad/s]
    float ang_rate[3] = {0, 0, 0};

    /// Yaw Pitch Roll 3-2-1 Euler angles with respect to NED. [deg]
    float ypr[3] = {0, 0, 0};

    /// Position: Latitude [deg], Longitude [deg], Altitude []
    double pos[3] = {0, 0, 0};

    /// Velocity in Body-Frame [m/s]
    float vel[3] = {0, 0, 0};

    uint8_t gnss_fix = 0;
};

enum class VectornavLink : uint8_t {
    Disconnected = 0,
    Configuring = 1,
    NoData = 2,
    Streaming = 3,
};

enum class VnError : uint8_t {
    None = 0,
    ResponseTimeout = 1,
    SensorError = 2,
    BadChecksum = 3,
};

// Fixed: the VectorNav SDK plus the C++ library code it pulls in (~100 KB flash) doesn't fit the
// STM32G441's 128 KB, so this talks to the VN-200 directly: a few ASCII commands to set it up, then
// decoding the two binary packets we configure
class VectorNavIMU {
public:
    static constexpr uint32_t DEFAULT_BAUD = 115200;
    static constexpr uint32_t FAST_BAUD = 921600;

    VectorNavIMU(PinName tx, PinName rx, PinName restore, PinName power);

    /** Gets the latest data from the sensor AND puts it in the state, run once per cycle or
     * whenever you need data
     */
    void update_state(VectornavState& state);

    VectornavLink link() const { return link_state; }
    uint32_t baud() const { return current_baud; }
    uint16_t configure_count() const { return configure_attempts; }
    uint16_t error_count() const { return errors; }
    VnError last_error() const { return latest_error; }
    uint8_t last_sensor_error() const { return latest_sensor_error; }
    const char* model() const { return model_number; }

private:
    enum class Step : uint8_t { PowerOff, WaitBoot, Find, SetBaud, Configure, Streaming };
    enum class Response : uint8_t { None, Ok, Error };

    static constexpr uint32_t POWER_OFF_MS = 500;
    static constexpr uint32_t BOOT_TIMEOUT_MS = 10000;
    static constexpr uint32_t BOOT_LISTEN_MS = 250;
    static constexpr uint32_t RESPONSE_TIMEOUT_MS = 250;
    static constexpr uint32_t BAUD_SWITCH_MS = 50;
    // Fixed: start() used to run once at boot so if the sensor wasn't answering yet the IMU
    // stayed dead until a reboot. added retry if not connected
    static constexpr uint32_t RETRY_PERIOD_MS = 1000;
    // Fixed: outputs aren't saved to the sensor's flash (after talking to Luca), so after it
    // power-cycles it goes quiet. If timeout, reconfigure IMU
    static constexpr uint32_t DATA_TIMEOUT_MS = 500;
    static constexpr uint8_t FIND_ATTEMPTS = 8;

    BufferedSerial serial;
    DigitalOut restore_pin;
    DigitalOut power_pin;

    Step step = Step::PowerOff;
    bool step_started = false;
    uint32_t step_time = 0;
    uint32_t baud_toggle_time = 0;
    uint8_t attempts = 0;
    uint8_t config_index = 0;
    bool waiting = false;
    bool booted = false;
    bool retry_pending = false;
    Response response = Response::None;
    const char* expected = nullptr;
    uint32_t sent_time = 0;

    uint32_t current_baud = DEFAULT_BAUD;
    VectornavLink link_state = VectornavLink::Disconnected;
    uint32_t last_packet = 0;
    uint16_t configure_attempts = 0;
    uint16_t errors = 0;
    VnError latest_error = VnError::None;
    uint8_t latest_sensor_error = 0;
    char model_number[24] = "";

    enum class Rx : uint8_t { Idle, Ascii, Binary };
    Rx rx = Rx::Idle;
    char line[128];
    size_t line_len = 0;
    uint8_t packet[64];
    size_t packet_len = 0;
    size_t packet_expected = 0;

    static uint32_t millis();
    void set_baud(uint32_t baud);

    void enter(Step next);
    void send(const char* body, const char* expect);
    void retry_later(VnError err);
    void record_error(VnError err);
    void handle_line(VectornavState& state);
    void receive(uint8_t byte, VectornavState& state);
    void decode_packet(VectornavState& state);
};

namespace vn {
uint8_t checksum(const char* data, size_t len);
uint16_t crc16(const uint8_t* data, size_t len);
bool valid_line(const char* line, size_t len);
} // namespace vn
