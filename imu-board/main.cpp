#include "mbed.h"
#include "vectornav_imu.h"
#include <cstdio>

VectorNavIMU imu{PC_12, PD_2}; // TX = PC_12, RX = PD_2
DigitalOut status_led{LED1};
VectornavState vn_state;

void print_state();

namespace {
static constexpr auto PRINT_PERIOD = 100ms;
}

int main() {
    printf("Hello World!!\n");

    // Fixed: imu.start() ran on the main thread while the IMU thread was already calling
    // update_state(), so two threads used the sensor at once
    auto next_print = Kernel::Clock::now();
    while (true) {
        imu.update_state(vn_state);

        auto now = Kernel::Clock::now();
        if (now >= next_print) {
            print_state();
            status_led = (imu.link() == VectornavLink::Streaming) ? 1 : !status_led;
            next_print = now + PRINT_PERIOD;
        }

        ThisThread::sleep_for(1ms);
    }
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
    printf("ypr   [deg]   %f %f %f\n", vn_state.ypr.yaw, vn_state.ypr.pitch, vn_state.ypr.roll);
    printf("vel   [m/s]   %f %f %f\n", vn_state.vel[0], vn_state.vel[1], vn_state.vel[2]);
    printf("lla           %f %f %f | gnss fix %u\n",
           vn_state.pos.lat, vn_state.pos.lon, vn_state.pos.alt, vn_state.gnss_fix);
}
