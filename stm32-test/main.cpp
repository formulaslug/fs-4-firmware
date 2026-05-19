#include "mbed.h"
#include "vectornav_imu.h"

// CAN can{PA_11, PA_12, 500000};
// tx,rx,baud
VectorNavIMU imu(PC_12, PD_2);

int main() {
    printf("Hello World!\n");

    VN::Error err = imu.connect();
    if (err != VN::Error::None) {
        CHECK_VN_ERR(err);
        while (1) ThisThread::sleep_for(10000ms);
    }
    printf("Connected to sensor!\n");

    printf("Sensor Model Number: %s\n", imu.getModel());

    printf("baud: %d\n", imu.getConnectedBaudRate());

    imu.initRegisters();
    imu.enableRawImu(true);
    imu.enableAttitude(true);
    imu.enableNavigation(true);
    imu.enableGpsTime(true);
    imu.enableUncertainty(true);
    imu.enableStatus(true);
    err = imu.applyRegisters();
    if (err != VN::Error::None) {
        CHECK_VN_ERR(err);
        while (1) ThisThread::sleep_for(10000ms);
    }

    while (1) {
        imu.refreshData();

        std::optional<VN::Vec3f> accel = imu.getAccel();
        if (accel.has_value()) {
            printf("\tAccel X: %f\n\tAccel Y: %f\n\t Accel Z: %f\n", accel.value()[0], accel.value()[1], accel.value()[2]);
        }

        std::optional<VN::Vec3f> angRate = imu.getAngularRate();
        if (angRate.has_value()) {
            printf("\tAng X: %f\n\tAng Y: %f\n\t Ang Z: %f\n", angRate.value()[0], angRate.value()[1], angRate.value()[2]);
        }

        std::optional<VN::Vec3f> mag = imu.getMag();
        if (mag.has_value()) {
            printf("\tMag X: %f\n\tMag Y: %f\n\t Mag Z: %f\n", mag.value()[0], mag.value()[1], mag.value()[2]);
        }

        std::optional<VN::Ypr> ypr = imu.getYawPitchRoll();
        if (ypr.has_value()) {
            printf("\tYaw: %f\n\tPitch: %f\n\t Roll: %f\n", ypr.value().yaw, ypr.value().pitch, ypr.value().roll);
        }

        std::optional<VN::Lla> lla = imu.getPosition();
        if (lla.has_value()) {
            printf("\tLat: %f\n\tLon: %f\n\t Alt: %f\n", lla.value().lat, lla.value().lon, lla.value().alt);
        }

        std::optional<VN::Vec3f> vel = imu.getVelocityBody();
        if (vel.has_value()) {
            printf("\tVel X: %f\n\tVel Y: %f\n\t Vel Z: %f\n", vel.value()[0], vel.value()[1], vel.value()[2]);
        }

        // Handle asynchronous errors
        std::optional<VN::AsyncError> asyncError = imu.getAsyncError();
        if (asyncError.has_value()) {
            printf("Received async error: %s\n", asyncError.value().message.data());
        }
    }

    imu.disconnect();

    while (1) {};
}
