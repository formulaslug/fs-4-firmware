#include "mbed.h"

using namespace std::chrono_literals;

// #define BNO_RST PC_8
// #define BNO_INT PB_13
// #define BNO_WAKE PA_11
// #define BNO_MISO PB_14
// #define BNO_MOSI PB_15
// #define BNO_SCK PC_7
// #define BNO_CS PB_12

#define BNO_USE_SPI
#include "7Semi_BNO08x.h"

// constexpr int PIN_SCK = -1;
// constexpr int PIN_MISO = -1;
// constexpr int PIN_MOSI = -1;
// constexpr int PIN_CS = 10;
// constexpr int PIN_INTN = 9; // Optional for SPI
// constexpr int PIN_RST = 8;  // Optional for SPI

/* ---------------- SPI Bus ---------------- */
BnoSPIBus bus(PB_12, PB_13, PC_8, PA_11, 1000000UL, 3, PC_7, PB_14, PB_15);
// BnoSPIBus bus(PA_11, PB_5, PA_8, PB_1, 1000000UL, 3, PA_5, PA_6, PA_7); // nucleo l432kc

/* ---------------- IMU ---------------- */
BNO08x_7Semi bno(bus);

/* ---------------- Cached IMU Data ---------------- */
struct ImuData {
    float ax, ay, az;     // Accelerometer
    float gx, gy, gz;     // Gyroscope
    float mx, my, mz;     // Magnetometer
    float qi, qj, qk, qr; // Rotation Vector
    float lax, lay, laz;  // Linear acceleration
};

ImuData d;
static uint64_t t = 0;

int main() {
    printf("[BNO08x SPI STM32]\n");

    if (!bno.begin()) {
        printf("BNO08x initialization failed\n");
        while (1)
            ThisThread::sleep_for(1000ms);
    }
    printf("BNO08x ready\n");

    bool reportsOk = true;
    reportsOk = bno.enableAcc(20) && reportsOk;
    reportsOk = bno.enableGyro(20) && reportsOk;
    reportsOk = bno.enableMag(50) && reportsOk;
    reportsOk = bno.enableRotationVector(20) && reportsOk;
    reportsOk = bno.enableLinearAccel(20) && reportsOk;
    if (!reportsOk) {
        printf("One or more BNO08x reports failed to enable\n");
    } else {
        printf("BNO08x reports requested\n");
    }

    bool sawData = false;
    bool noDataWarned = false;
    bool reportsRetried = false;
    while (true) {
        bno.processData(); // Always run as fast as possible
        if (Kernel::get_ms_count() - t > 250) {
            t = Kernel::get_ms_count();
            bool printed = false;
            if (bno.getAccelerometer(d.ax, d.ay, d.az)) {
                printf("ACC : %f %f %f\n", d.ax, d.ay, d.az);
                printed = true;
            }
            if (bno.getGyroscope(d.gx, d.gy, d.gz)) {
                printf("GYRO: %f %f %f\n", d.gx, d.gy, d.gz);
                printed = true;
            }
            if (bno.getMagnetometer(d.mx, d.my, d.mz)) {
                printf("MAG: %f %f %f\n", d.mx, d.my, d.mz);
                printed = true;
            }
            if (bno.getQuaternion(d.qi, d.qj, d.qk, d.qr)) {
                printf("RV: %f %f %f %f\n", d.qi, d.qj, d.qk, d.qr);
                printed = true;
            }
            if (bno.getLinearAccel(d.lax, d.lay, d.laz)) {
                printf("LIN: %f %f %f\n", d.lax, d.lay, d.laz);
                printed = true;
            }
            if (printed) {
                sawData = true;
                printf("\n");
            } else if (!sawData && !noDataWarned && Kernel::get_ms_count() > 2000) {
                noDataWarned = true;
                printf("No BNO08x sensor data yet\n");
            } else if (!sawData && !reportsRetried && Kernel::get_ms_count() > 4000) {
                reportsRetried = true;
                printf("Re-requesting BNO08x reports\n");
                bool retryOk = true;
                retryOk = bno.enableAcc(20) && retryOk;
                retryOk = bno.enableGyro(20) && retryOk;
                retryOk = bno.enableMag(50) && retryOk;
                retryOk = bno.enableRotationVector(20) && retryOk;
                retryOk = bno.enableLinearAccel(20) && retryOk;
                if (!retryOk) {
                    printf("BNO08x report retry failed\n");
                }
            }
        }
        ThisThread::sleep_for(1ms);
    }
}
