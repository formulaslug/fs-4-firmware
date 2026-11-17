#include "mbed.h"
#include "vectornav_imu.h"
#include <cstdio>

VectorNavIMU imu{PA_9, PA_10, PA_6, PB_5}; // TX = PA_9, RX = PA_10, RESTORE = PA_6, POWER = PB_5 (VN_EN)
CAN can{PA_11, PA_12, 1000000};
VectornavState vn_state;

// Fixed: the VCU wrote all 5 frames at once into 3 TX mailboxes
// so its possible the extras were dropped.
CircularBuffer<CANMessage, 10> can_queue;

void send_imu_CAN_messages();
void print_state();

namespace {
static constexpr auto CAN_PERIOD = 10ms;
static constexpr auto PRINT_PERIOD = 100ms;
static constexpr float RAD_TO_DEG = 57.2957795f;
}

int main() {
    printf("Hello World!!\n");

    // Fixed: imu.start() ran on the main thread while the IMU thread was already calling
    // update_state(), so two threads used the sensor at once
    auto next_can = Kernel::Clock::now();
    auto next_print = Kernel::Clock::now();
    while (true) {
        imu.update_state(vn_state);

        auto now = Kernel::Clock::now();
        if (now >= next_can) {
            // Fixed: the VCU sent these even while the IMU wasn't streaming
            if (imu.link() == VectornavLink::Streaming) send_imu_CAN_messages();
            next_can = now + CAN_PERIOD;
        }

        CANMessage msg;
        while (can_queue.peek(msg) && can.write(msg) == 1) can_queue.pop(msg);

        if (now >= next_print) {
            print_state();
            next_print = now + PRINT_PERIOD;
        }

        ThisThread::sleep_for(1ms);
    }
}

// 100Hz VectorNav Messages
void send_imu_CAN_messages() {
    uint8_t buf_accel[6];
    int16_t ax = static_cast<int16_t>(vn_state.accel[0] * 100);
    int16_t ay = static_cast<int16_t>(vn_state.accel[1] * 100);
    int16_t az = static_cast<int16_t>(vn_state.accel[2] * 100);
    buf_accel[0] = ax & 0xFF;
    buf_accel[1] = ax >> 8;
    buf_accel[2] = ay & 0xFF;
    buf_accel[3] = ay >> 8;
    buf_accel[4] = az & 0xFF;
    buf_accel[5] = az >> 8;

    uint8_t buf_gyro[6];
    int16_t wx = static_cast<int16_t>(vn_state.ang_rate[0] * RAD_TO_DEG * 10);
    int16_t wy = static_cast<int16_t>(vn_state.ang_rate[1] * RAD_TO_DEG * 10);
    int16_t wz = static_cast<int16_t>(vn_state.ang_rate[2] * RAD_TO_DEG * 10);
    buf_gyro[0] = wx & 0xFF;
    buf_gyro[1] = wx >> 8;
    buf_gyro[2] = wy & 0xFF;
    buf_gyro[3] = wy >> 8;
    buf_gyro[4] = wz & 0xFF;
    buf_gyro[5] = wz >> 8;

    uint8_t buf_ypr[6];
    int16_t yaw = static_cast<int16_t>(vn_state.ypr[0] * 100);
    int16_t pitch = static_cast<int16_t>(vn_state.ypr[1] * 100);
    int16_t roll = static_cast<int16_t>(vn_state.ypr[2] * 100);
    buf_ypr[0] = yaw & 0xFF;
    buf_ypr[1] = yaw >> 8;
    buf_ypr[2] = pitch & 0xFF;
    buf_ypr[3] = pitch >> 8;
    buf_ypr[4] = roll & 0xFF;
    buf_ypr[5] = roll >> 8;

    uint8_t buf_vel[7];
    int16_t vx = static_cast<int16_t>(vn_state.vel[0] * 100);
    int16_t vy = static_cast<int16_t>(vn_state.vel[1] * 100);
    int16_t vz = static_cast<int16_t>(vn_state.vel[2] * 100);
    buf_vel[0] = vx & 0xFF;
    buf_vel[1] = vx >> 8;
    buf_vel[2] = vy & 0xFF;
    buf_vel[3] = vy >> 8;
    buf_vel[4] = vz & 0xFF;
    buf_vel[5] = vz >> 8;
    buf_vel[6] = vn_state.gnss_fix;

    uint8_t buf_latlon[8];
    int32_t lat = static_cast<int32_t>(vn_state.pos[0] * 1e7);
    int32_t lon = static_cast<int32_t>(vn_state.pos[1] * 1e7);
    buf_latlon[0] = lat & 0xFF;
    buf_latlon[1] = (lat >> 8) & 0xFF;
    buf_latlon[2] = (lat >> 16) & 0xFF;
    buf_latlon[3] = (lat >> 24) & 0xFF;
    buf_latlon[4] = lon & 0xFF;
    buf_latlon[5] = (lon >> 8) & 0xFF;
    buf_latlon[6] = (lon >> 16) & 0xFF;
    buf_latlon[7] = (lon >> 24) & 0xFF;

    CANMessage accel_msg{0x2D0, buf_accel, 6};
    CANMessage gyro_msg{0x3D1, buf_gyro, 6};
    CANMessage ypr_msg{0x3D0, buf_ypr, 6};
    CANMessage vel_msg{0x2D2, buf_vel, 7};
    CANMessage latlon_msg{0x2D1, buf_latlon, 8};

    can_queue.push(accel_msg);
    can_queue.push(gyro_msg);
    can_queue.push(ypr_msg);
    can_queue.push(vel_msg);
    can_queue.push(latlon_msg);
}

void print_state() {
    static constexpr const char* LINK_NAMES[] = {"disconnected", "configuring", "no data", "streaming"};

    printf("\n[%s] %lu baud | configs %u, errors %u (last %u)\n",
           LINK_NAMES[static_cast<uint8_t>(imu.link())],
           imu.baud(),
           imu.configure_count(),
           imu.error_count(),
           static_cast<uint16_t>(imu.last_error()));
    printf("accel [m/s^2] %f %f %f\n", vn_state.accel[0], vn_state.accel[1], vn_state.accel[2]);
    printf(
        "gyro  [rad/s] %f %f %f\n", vn_state.ang_rate[0], vn_state.ang_rate[1], vn_state.ang_rate[2]
    );
    printf("ypr   [deg]   %f %f %f\n", vn_state.ypr[0], vn_state.ypr[1], vn_state.ypr[2]);
    printf("vel   [m/s]   %f %f %f\n", vn_state.vel[0], vn_state.vel[1], vn_state.vel[2]);
    printf("lla           %f %f %f | gnss fix %u\n",
           vn_state.pos[0], vn_state.pos[1], vn_state.pos[2], vn_state.gnss_fix);
}
