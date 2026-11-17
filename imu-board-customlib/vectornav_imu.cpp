#include "vectornav_imu.h"
#include <cstdio>
#include <cstring>

namespace {

constexpr uint8_t SYNC = 0xFA;

// Fixed: originally 100Hz (IMU), 25Hz (INS) and 100Hz (attitude),
// original the rate comments had the math wrong
// accel/gyro run at the full 800Hz IMU rate and attitude/INS at the
// VN-200's 400Hz max in one packet
constexpr const char* CONFIG_COMMANDS[][2] = {
    {"VNWRG,06,0", "$VNWRG,06"},
    {"VNWRG,75,2,1,04,0600", "$VNWRG,75"},
    {"VNWRG,76,2,2,38,0010,0002,000A", "$VNWRG,76"},
};
constexpr size_t CONFIG_COUNT = sizeof(CONFIG_COMMANDS) / sizeof(CONFIG_COMMANDS[0]);

constexpr uint8_t IMU_GROUPS = 0x04;
constexpr uint16_t IMU_FIELDS = 0x0600;
constexpr size_t IMU_PACKET_LEN = 1 + 1 + 2 + 12 + 12 + 2;

constexpr uint8_t NAV_GROUPS = 0x38;
constexpr uint16_t NAV_GNSS_FIELDS = 0x0010;
constexpr uint16_t NAV_ATTITUDE_FIELDS = 0x0002;
constexpr uint16_t NAV_INS_FIELDS = 0x000A;
constexpr size_t NAV_PACKET_LEN = 1 + 1 + 6 + 1 + 12 + 24 + 12 + 2;

uint16_t le16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

template <typename T> T get(const uint8_t* p) {
    T value;
    memcpy(&value, p, sizeof(T));
    return value;
}

// Wraps angle to [-180, 180)
float wrap_angle(float angle) {
    if (angle >= 180.0f) {
        return angle - 360.0f;
    } else if (angle < -180.0f) {
        return angle + 360.0f;
    }
    return angle;
}

int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool starts_with(const char* s, const char* prefix) { return strncmp(s, prefix, strlen(prefix)) == 0; }

} // namespace

namespace vn {

uint8_t checksum(const char* data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) sum ^= static_cast<uint8_t>(data[i]);
    return sum;
}

uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = static_cast<uint16_t>((crc >> 8) | (crc << 8));
        crc ^= data[i];
        crc ^= static_cast<uint8_t>(crc & 0xFF) >> 4;
        crc ^= static_cast<uint16_t>(crc << 12);
        crc ^= static_cast<uint16_t>((crc & 0x00FF) << 5);
    }
    return crc;
}

bool valid_line(const char* line, size_t len) {
    if (len < 4 || line[0] != '$') return false;
    const char* star = static_cast<const char*>(memchr(line, '*', len));
    if (star == nullptr || star + 3 > line + len) return false;
    if (star[1] == 'X' && star[2] == 'X') return true;
    int hi = hex_digit(star[1]);
    int lo = hex_digit(star[2]);
    if (hi < 0 || lo < 0) return false;
    return checksum(line + 1, static_cast<size_t>(star - line - 1)) == ((hi << 4) | lo);
}

} // namespace vn

void VectorNavIMU::enter(Step next) {
    step = next;
    step_started = false;
}

void VectorNavIMU::record_error(VnError err) {
    if (errors < UINT16_MAX) errors++;
    latest_error = err;
}

void VectorNavIMU::send(const char* body, const char* expect) {
    char cmd[64];
    int len = snprintf(cmd, sizeof(cmd), "$%s*XX\r\n", body);
    serial.write(cmd, static_cast<size_t>(len));
    expected = expect;
    waiting = true;
    sent_time = millis();
}

void VectorNavIMU::retry_later(VnError err) {
    record_error(err);
    printf("VN Error: %s\n", err == VnError::SensorError ? "sensor replied with an error" : "ResponseTimeout");
    waiting = false;
    enter(Step::Find);
    step_time = millis();
    retry_pending = true;
}

void VectorNavIMU::receive(uint8_t byte, VectornavState& state) {
    switch (rx) {
    case Rx::Idle:
        if (byte == '$') {
            line[0] = '$';
            line_len = 1;
            rx = Rx::Ascii;
        } else if (byte == SYNC) {
            packet[0] = SYNC;
            packet_len = 1;
            packet_expected = 0;
            rx = Rx::Binary;
        }
        break;

    case Rx::Ascii:
        if (byte == '\n' || byte == '\r') {
            line[line_len] = '\0';
            if (vn::valid_line(line, line_len)) handle_line(state);
            rx = Rx::Idle;
        } else if (line_len + 1 < sizeof(line)) {
            line[line_len++] = static_cast<char>(byte);
        } else {
            rx = Rx::Idle;
        }
        break;

    case Rx::Binary:
        packet[packet_len++] = byte;
        if (packet_len == 2) {
            if (byte == IMU_GROUPS) {
                packet_expected = IMU_PACKET_LEN;
            } else if (byte == NAV_GROUPS) {
                packet_expected = NAV_PACKET_LEN;
            } else {
                rx = Rx::Idle;
            }
        } else if (packet_len == packet_expected) {
            decode_packet(state);
            rx = Rx::Idle;
        }
        break;
    }
}

void VectorNavIMU::decode_packet(VectornavState& state) {
    if (vn::crc16(packet + 1, packet_len - 1) != 0) return;

    if (packet[1] == IMU_GROUPS) {
        if (le16(packet + 2) != IMU_FIELDS) return;
        for (int i = 0; i < 3; i++) {
            state.accel[i] = get<float>(packet + 4 + 4 * i);
            state.ang_rate[i] = get<float>(packet + 16 + 4 * i);
        }
    } else {
        if (le16(packet + 2) != NAV_GNSS_FIELDS || le16(packet + 4) != NAV_ATTITUDE_FIELDS
            || le16(packet + 6) != NAV_INS_FIELDS) {
            return;
        }
        state.gnss_fix = packet[8];
        for (int i = 0; i < 3; i++) {
            state.ypr[i] = wrap_angle(get<float>(packet + 9 + 4 * i));
            state.pos[i] = get<double>(packet + 21 + 8 * i);
            state.vel[i] = get<float>(packet + 45 + 4 * i);
        }
    }

    last_packet = millis();
    if (step == Step::WaitBoot) booted = true;
    if (step == Step::Streaming) link_state = VectornavLink::Streaming;
}

void VectorNavIMU::handle_line(VectornavState&) {
    if (step == Step::WaitBoot) {
        booted = true;
        return;
    }
    if (!waiting) return;

    if (starts_with(line, "$VNERR")) {
        const char* code = line + 7;
        latest_sensor_error = static_cast<uint8_t>((hex_digit(code[0]) << 4) | hex_digit(code[1]));
        response = Response::Error;
        waiting = false;
    } else if (expected != nullptr && starts_with(line, expected)) {
        if (starts_with(line, "$VNRRG,01,")) {
            const char* start = line + 10;
            const char* end = strchr(start, '*');
            size_t n = end ? static_cast<size_t>(end - start) : 0;
            if (n >= sizeof(model_number)) n = sizeof(model_number) - 1;
            memcpy(model_number, start, n);
            model_number[n] = '\0';
        }
        response = Response::Ok;
        waiting = false;
    }
}

VectorNavIMU::VectorNavIMU(PinName tx, PinName rx, PinName restore, PinName power)
    : serial{tx, rx, DEFAULT_BAUD}, restore_pin{restore, 1}, power_pin{power, 0} {
    serial.set_blocking(false);
}

uint32_t VectorNavIMU::millis() { return static_cast<uint32_t>(Kernel::Clock::now().time_since_epoch().count()); }

void VectorNavIMU::set_baud(uint32_t baud) { serial.set_baud(static_cast<int>(baud)); }

void VectorNavIMU::update_state(VectornavState& state) {
    uint8_t buf[256];
    ssize_t n;
    while ((n = serial.read(buf, sizeof(buf))) > 0) {
        for (ssize_t i = 0; i < n; i++) receive(buf[i], state);
    }

    const uint32_t now = millis();
    const bool timed_out = waiting && now - sent_time > RESPONSE_TIMEOUT_MS;
    const Response got = response;
    response = Response::None;

    switch (step) {
    // Fixed: when only the STM32 reset or boots up, the sensor was still at whatever baud it was
    // last switched to. The VN-200 restores its factory settings (115200 baud) if RESTORE is high
    // while it powers on, so hold RESTORE high and power-cycle it through the power switch.
    case Step::PowerOff:
        if (!step_started) {
            step_started = true;
            step_time = now;
            link_state = VectornavLink::Disconnected;
            restore_pin = 1;
            power_pin = 0;
        } else if (now - step_time >= POWER_OFF_MS) {
            current_baud = DEFAULT_BAUD;
            set_baud(current_baud);
            power_pin = 1;
            enter(Step::WaitBoot);
        }
        break;

    // Fixed: the sensor varies 4-6s to restart after factory reset. Asking it
    // anything while resetting gets answered late, so wait for its output instead
    case Step::WaitBoot:
        if (!step_started) {
            step_started = true;
            step_time = now;
            baud_toggle_time = now;
            booted = false;
        } else if (booted || now - step_time >= BOOT_TIMEOUT_MS) {
            printf("Restored sensor factory settings\n");
            enter(Step::Find);
        } else if (now - baud_toggle_time >= BOOT_LISTEN_MS) {
            baud_toggle_time = now;
            current_baud = current_baud == FAST_BAUD ? DEFAULT_BAUD : FAST_BAUD;
            set_baud(current_baud);
        }
        break;

    // Fixed: sensor is at DEFAULT_BAUD after it powers up, but still at FAST_BAUD if only this
    // board reset, so if it doesn't answer at our current baud, try the other one.
    case Step::Find:
        if (!step_started) {
            if (retry_pending && now - step_time < RETRY_PERIOD_MS) break;
            retry_pending = false;
            step_started = true;
            attempts = 0;
            configure_attempts++;
            link_state = VectornavLink::Configuring;
            send("VNRRG,01", "$VNRRG,01,");
        } else if (got == Response::Ok) {
            printf("Sensor Model Number: %s\n", model_number);
            printf("baud: %lu\n", static_cast<unsigned long>(current_baud));
            enter(current_baud == FAST_BAUD ? Step::Configure : Step::SetBaud);
        } else if (got == Response::Error || timed_out) {
            if (++attempts >= FIND_ATTEMPTS) {
                record_error(VnError::ResponseTimeout);
                printf("No answer from sensor, power-cycling it\n");
                waiting = false;
                enter(Step::PowerOff);
                break;
            }
            uint32_t other = current_baud == FAST_BAUD ? DEFAULT_BAUD : FAST_BAUD;
            printf("No answer at %lu baud, trying %lu\n", static_cast<unsigned long>(current_baud),
                   static_cast<unsigned long>(other));
            current_baud = other;
            set_baud(current_baud);
            send("VNRRG,01", "$VNRRG,01,");
        }
        break;

    case Step::SetBaud:
        if (!step_started) {
            step_started = true;
            step_time = 0;
            send("VNWRG,05,921600", "$VNWRG,05");
        } else if (got == Response::Ok) {
            step_time = now;
        } else if (got == Response::Error || timed_out) {
            retry_later(got == Response::Error ? VnError::SensorError : VnError::ResponseTimeout);
        } else if (step_time != 0 && now - step_time >= BAUD_SWITCH_MS) {
            current_baud = FAST_BAUD;
            set_baud(current_baud);
            printf("baud: %lu\n", static_cast<unsigned long>(current_baud));
            enter(Step::Configure);
        }
        break;

    case Step::Configure:
        if (!step_started) {
            step_started = true;
            config_index = 0;
            send(CONFIG_COMMANDS[0][0], CONFIG_COMMANDS[0][1]);
        } else if (got == Response::Ok) {
            if (++config_index == CONFIG_COUNT) {
                enter(Step::Streaming);
            } else {
                send(CONFIG_COMMANDS[config_index][0], CONFIG_COMMANDS[config_index][1]);
            }
        } else if (got == Response::Error || timed_out) {
            retry_later(got == Response::Error ? VnError::SensorError : VnError::ResponseTimeout);
        }
        break;

    case Step::Streaming:
        if (!step_started) {
            step_started = true;
            last_packet = now;
            link_state = VectornavLink::NoData;
        } else if (now - last_packet > DATA_TIMEOUT_MS) {
            printf("No IMU data for %lu ms, reconfiguring\n", static_cast<unsigned long>(now - last_packet));
            enter(Step::Find);
        } else if (now - last_packet > 100) {
            link_state = VectornavLink::NoData;
        }
        break;
    }
}
