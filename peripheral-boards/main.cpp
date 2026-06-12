#include "ThisThread.h"
#include "config.hpp"
#include "d6t-1a.h"
#include "d6t-8lh.h"
#include "mbed.h"
#include "wheel_speed.hpp"
#include <cstring>
// #include "strain_gauge_235sl.h" // TODO: Uncomment when StrainGuage PR is merged

CAN can{PIN_CAN1_RX, PIN_CAN1_TX, CAN_FREQUENCY};
WheelSpeed wheelSensor{PIN_WHEEL_SENSOR, TEETH_PER_REV};
DigitalIn wheelPin{PIN_WHEEL_SENSOR};
AnalogIn sus{PIN_SUSPENSION};
I2C i2c{PIN_I2C2_SDA, PIN_I2C2_SCL};
D6T8LH d6t8{i2c};
I2C i2c1{PIN_I2C1_SDA, PIN_I2C1_SCL};
D6T1A d6t1{i2c1};
// StrainGauge235SL sg{PIN_STRAIN}; // TODO: Uncomment when StrainGuage PR is merged
// Suggestion: Set the output to also include ticks since the last can message was sent
//  Not really sure what to do with this output yet
Timer canMsgTimer;
CornerConfig cfg;
uint64_t last_sent_temp = 0;
uint64_t last_sent_tpdo = 0;

// tbh, I kind of just clauded the multithreading, but I'll be rereading and checking this later
// Two threads, when the wheelspeed interrupt happens, the thread with the highest priority gets returned to since we are using mbed
// === CAN TX thread =============================================================
// The D6T sensors are read over blocking I2C: mbed's STM32 driver busy-waits up to
// ~300ms-1s on a slow/failed transfer. If that ran in the same thread as the periodic
// CAN sends, it would freeze the 100Hz frames for the whole timeout. 

EventQueue canQueue{EVENTS_EVENT_SIZE * 16}; //Slow sensor reads run on the canQueue thread and the RTOS preempts them when CAN is due.
Thread canThread{osPriorityHigh, OS_STACK_SIZE, nullptr, "can_tx"}; //CAN TX runs on its own high-priority thread/queue and only does fast, non-blocking work;
uint8_t can_tx_fail_count = 0; // Consecutive CAN TX failures; used for conditional bus-off recovery (CAN-thread only)
constexpr uint8_t CAN_TX_FAIL_THRESHOLD = 5;

// === Sensor reads (thread) =============================================================
EventQueue sensorQueue{EVENTS_EVENT_SIZE * 16};
uint8_t i2c_fail_count = 0;  // sensor-thread only
uint8_t i2c1_fail_count = 0; // sensor-thread only

// Latest sensor values, written by the sensor thread and read by the CAN thread.
// Guarded by sensorMutex (held only for the brief copy, never during I2C).
Mutex sensorMutex;
uint8_t shared_pixels8lh[D6T8LH::N_PIXEL] = {0};
uint8_t shared_side_temp = 0; // D6T-1A side tire temp (DATA_SIDE_TIRE_TEMP)

/**
 * @brief CAN message sending, just sends the can message. If it fails too many times, resets the CAN object
 */
void sendCANmessage(const CANMessage& msg) {
    if (can.write(msg)) {
        can_tx_fail_count = 0;
    } else if (++can_tx_fail_count >= CAN_TX_FAIL_THRESHOLD) {
        can.reset();
        can_tx_fail_count = 0;
    }
}

/**
 * @brief reads the temperature sensors. Runs on a different thread since when the I2C is blocked, it stalls the CAN reads
 */
void readSensors() {
    uint8_t local8[D6T8LH::N_PIXEL] = {0};
    bool publish8 = false;
    if (cfg.has_tiretemp_1x8 && i2c_fail_count < 10) {
        // Per-read retry (manual section 6.4 measure() loop): a single NACK shouldn't drop the
        // whole cycle. On the sensor thread, so the delay can't disturb CAN timing.
        int read_result = 1;
        for (int attempt = 0; attempt < 3 && read_result != 0; attempt++) {
            read_result = d6t8.read();
            if (read_result != 0) ThisThread::sleep_for(2ms);
        }
        if (!read_result) {
            const double* px8 = d6t8.pixels_c();
            for (int i = 0; i < D6T8LH::N_PIXEL; i++) {
                local8[i] = (uint8_t)(px8[i]);
            }
            // TEMP DIAGNOSTIC: actual sensor values in degrees C (and internal PTAT temp).
            printf("d6t8 ok  ptat=%.1f  px=%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n",
                   d6t8.ptat_c(), px8[0], px8[1], px8[2], px8[3], px8[4], px8[5], px8[6], px8[7]);
            i2c_fail_count = 0; // Recover from transient failures
        } else {
            printf("Failed reading d6t8 : %d\n", read_result);
            local8[0] = 66; //  BAD DATA when sensor data is
            i2c_fail_count++;
        }
        publish8 = true;
    }

    uint8_t local1 = 0;
    bool publish1 = false;
    if (cfg.has_tiretemp_1x1 && i2c1_fail_count < 10) {
        if (d6t1.read()) {
            local1 = (uint8_t)d6t1.pixel_c();
            i2c1_fail_count = 0; // Recover from transient failures
            publish1 = true;
        } else {
            printf("Failed reading d6t1\n");
            i2c1_fail_count++;
        }
    }

    sensorMutex.lock();
    if (publish8) std::memcpy(shared_pixels8lh, local8, sizeof(shared_pixels8lh));
    if (publish1) shared_side_temp = local1;
    sensorMutex.unlock();
}

int main() {
    printf("main()\n");
    wheelPin.mode(PullUp);
    cfg = getCornerConfig(readCorner());

    // Set the D6T bus speed before any sensor I/O (see D6T_I2C_HZ in config.hpp; lower it if the
    // bus is marginal). Both D6T sensors are standard-mode (<=100 kHz) parts.
    i2c.frequency(D6T_I2C_HZ);
    i2c1.frequency(D6T_I2C_HZ);

    if (cfg.has_tiretemp_1x8 & !d6t8.setup()){
        printf("d68t init fail!!\n");
    } else {
        printf("d68t init success!!\n");
    }
    if (cfg.has_tiretemp_1x1 & !d6t1.setup()){
        printf("d6t1 init fail!!\n");
    } else {
        printf("d6t1 init success!!\n");
    }

    // TODO: Uncomment when StrainGuage PR is merged
    // //Strain Guage Setup
    // sg.set_filter_window(32);
    // ThisThread::sleep_for(500ms);
    // sg.tare(500, 200);
    // //apply calibration slope
    // //again this is a fake number, we'd have to calculate this
    // sg.set_calibration(2500.0f, 0.0f);
    // //End of Strain Guage Setup
    canMsgTimer.start();

    // CAN TX on a dedicated high-priority thread so blocking sensor I/O can't delay it.
    //canQueue.call_every(10ms, &sendCANtpdo);  // 100Hz
    canQueue.call_every(100ms, &sendCANtemp); // 10Hz
    canThread.start(callback(&canQueue, &EventQueue::dispatch_forever));

    // Sensor reads run on this (main) thread. A blocking/timed-out I2C read here no longer
    // affects CAN timing because the RTOS preempts this thread for the higher-priority CAN thread.
    sensorQueue.call_every(100ms, &readSensors); // 10Hz
    sensorQueue.dispatch_forever();

    return 0;
}
void sendLastMessageTicks() {
    uint64_t last_temp = canMsgTimer.elapsed_time().count() - last_sent_temp;
    uint64_t last_tpdo = canMsgTimer.elapsed_time().count() - last_sent_tpdo;
    // convert microsecond difference to ticks
    // 8 bits 0-256
    // Can probably be a 1 byte message
    // microseconds to 100hz
    uint64_t temp_ticks = (last_temp / 10000 > 15) ? 15 : last_temp / 10000;
    uint64_t tpdo_ticks = (last_tpdo / 10000 > 15) ? 15 : last_tpdo / 10000;
    // printf("Temp Ticks: %d\n Tpdo Ticks: %d\n", last_temp, last_tpdo);
    // Not sure how I would want to keep track of the ticks since last message
    // Right now it's in the same queue loop as everything else and I wonder if this is fine
    // Write and send this CAN Message
    // NOT FINISHED YET
}

void sendCANtemp() {
    // Snapshot the latest 8-pixel reading produced by the sensor thread (no I2C here).
    uint8_t pixels8lh[D6T8LH::N_PIXEL];

    sensorMutex.lock();
    std::memcpy(pixels8lh, shared_pixels8lh, sizeof(pixels8lh));
    sensorMutex.unlock();

    printf("Temp: %d, %d, %d, %d, %d, %d, %d, %d \n", pixels8lh[0],pixels8lh[1],pixels8lh[2],pixels8lh[3],pixels8lh[4],pixels8lh[5],pixels8lh[6],pixels8lh[7]);

    // Tire temperature message
    CANMessage tpdo_tiretemp_msg(cfg.tpdo_tiretemp_id, pixels8lh, 8);
    sendCANmessage(tpdo_tiretemp_msg);
    last_sent_temp = canMsgTimer.elapsed_time().count();
}

void sendCANtpdo() {
    uint16_t wheel_speed_raw = 0;
    uint16_t sus_travel_raw = 0;
    // 1 pixel side temp sensor (DATA_SIDE_TIRE_TEMP), produced by the sensor thread
    sensorMutex.lock();
    uint8_t px0 = shared_side_temp;
    sensorMutex.unlock();

    // Wheel Speed Readings
    // DATA_WHEELSPEED is signed 16-bit (scale 0.1), so clamp to int16 range before casting
    // to avoid wrapping negative above 3276.7 rpm.
    float wheel_speed_scaled = wheelSensor.update() * 10.0f; // scaled according to CAN.dbc values
    if (wheel_speed_scaled < -32768.0f) wheel_speed_scaled = -32768.0f;
    if (wheel_speed_scaled > 32767.0f) wheel_speed_scaled = 32767.0f;
    wheel_speed_raw = (int16_t)wheel_speed_scaled;

    // Suspension Travel Readings
    sus_travel_raw = ((1.0 - sus.read()) * 5000);
    // printf("Sus Travel: %d, ", sus_travel_raw);
    // printf("Sus Raw: %.4f \n ", sus.read());

    // switch (readCorner()) {
    // case Corner::FR:
    //     printf("FR\n");
    //     break;
    // case Corner::FL:
    //     printf("FL\n");
    //     break;
    // case Corner::BR:
    //     printf("BR\n");
    //     break;
    // case Corner::BL:
    //     printf("BL\n");
    //     break;
    // }
    // // TODO: Uncomment when StrainGuage PR is merged
    // // //Strain Guage Readings
    // // float force = sg.read_units();
    // // int16_t strain_raw = (int16_t)(force / 1e-7); //Scaled according to Can.dbc values

    // // Message Array
    uint8_t tpdo_data[] = {
        static_cast<uint8_t>(wheel_speed_raw & 0xFF),
        static_cast<uint8_t>((wheel_speed_raw & 0xFF00) >> 8),
        static_cast<uint8_t>(sus_travel_raw & 0xFF),
        static_cast<uint8_t>((sus_travel_raw & 0xFF00) >> 8),

        // TODO: Uncomment when StrainGuage PR is merged
        0x00,
        0x00,
        // strain_raw & 0xFF,
        // (strain_raw & 0xFF00) >> 8,
        px0,
    };

    CANMessage tpdo_msg(cfg.tpdo_data_id, tpdo_data, 7);
    // printf("%d:: WRITE START \n", canMsgTimer.elapsed_time().count());
    sendCANmessage(tpdo_msg);
    last_sent_tpdo = canMsgTimer.elapsed_time().count();

    //printf("CAN TX ERR CNT: %d\n", can.tderror());
    //printf("wheelspeed %d\n", wheel_speed_scaled);
}
