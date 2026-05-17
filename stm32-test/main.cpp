#include "mbed.h"

using namespace std::chrono_literals;

// #define BNO_RST PC_8
// #define BNO_INT PB_13
// #define BNO_WAKE PA_11
// #define BNO_MISO PB_14
// #define BNO_MOSI PB_15
// #define BNO_SCK PC_7
// #define BNO_CS PB_12

#include "7Semi_BNO08x.h"
#define BNO_USE_SPI

// constexpr int PIN_SCK = -1;
// constexpr int PIN_MISO = -1;
// constexpr int PIN_MOSI = -1;
// constexpr int PIN_CS = 10;
// constexpr int PIN_INTN = 9; // Optional for SPI
// constexpr int PIN_RST = 8;  // Optional for SPI

/* ---------------- SPI Bus ---------------- */
BnoSPIBus bus(PB_12, PB_13, PC_8, PA_11, 1000000UL, 3, PC_7, PB_14, PB_15);

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
static uint32_t t = 0;

int main() {
    DigitalOut chipSelect(PB_12);
    chipSelect = 1;

    printf("[BNO08x SPI ESP32]\n");

    if (!bno.begin()) {
        printf("BNO08x initialization failed\n");
        while (1)
            ThisThread::sleep_for(1000ms); 
    }

    bno.enableAcc(20);
    bno.enableGyro(20);
    bno.enableMag(50);
    bno.enableRotationVector(20);
    bno.enableLinearAccel(20);

    while (true) {
        printf("run\n");
        bno.processData(); // Always run as fast as possible
        if (Kernel::get_ms_count(); - t > 250) {
            t = Kernel::get_ms_count();;
            if (bno.getAccelerometer(d.ax, d.ay, d.az)) {
                printf("ACC : %f %f %f\n", d.ax, d.ay, d.az);
            }
            if (bno.getGyroscope(d.gx, d.gy, d.gz)) {
                printf("GYRO: %f %f %f\n", d.gx, d.gy, d.gz);
            }
            if (bno.getMagnetometer(d.mx, d.my, d.mz)) {
                printf("MAG: %f %f %f\n", d.mx, d.my, d.mz);
            }
            if (bno.getQuaternion(d.qi, d.qj, d.qk, d.qr)) {
                printf("RV: %f %f %f\n", d.qi, d.qj, d.qk);
            }
            if (bno.getLinearAccel(d.lax, d.lay, d.laz)) {
                printf("LIN: %f %f %f\n", d.lax, d.lay, d.laz);
            }
            printf("\n");
        }
    }
}
