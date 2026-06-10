#include "ThisThread.h"
#include "config.hpp"
#include "d6t-1a.h"
#include "d6t-8lh.h"
#include "mbed.h"
#include "wheel_speed.hpp"
// #include "strain_gauge_235sl.h" // TODO: Uncomment when StrainGuage PR is merged

CAN can{PIN_CAN1_RX, PIN_CAN1_TX, CAN_FREQUENCY};
WheelSpeed wheelSensor{PIN_WHEEL_SENSOR, TEETH_PER_REV};
DigitalIn wheelPin{PIN_WHEEL_SENSOR};
AnalogIn sus{PIN_SUSPENSION};
I2C i2c{PIN_I2C2_SDA, PIN_I2C2_SCL};
D6T8LH d6t8{i2c};
//D6T1A d6t1{i2c};
// StrainGauge235SL sg{PIN_STRAIN}; // TODO: Uncomment when StrainGuage PR is merged
// Suggestion: Set the output to also include ticks since the last can message was sent
//  Not really sure what to do with this output yet
Timer canMsgTimer;
CornerConfig cfg;
EventQueue queue = EventQueue{EVENTS_EVENT_SIZE * 32};
uint64_t last_sent_temp = 0;
uint64_t last_sent_tpdo = 0;
uint8_t i2c_fail_count = 0;

int main() {
    printf("main()\n");
    wheelPin.mode(PullUp);
    cfg = getCornerConfig(readCorner());
    if (!d6t8.setup()){
        printf("d68t init fail!!\n");
    } else {
        printf("d68t init success!!");
    }
    //d6t1.setup();

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
    queue.call_every(10ms, &sendCANtpdo);  // 100Hz
    queue.call_every(100ms, &sendCANtemp); // 10Hz
    //queue.call_every(100ms, &sendLastMessageTicks);
    queue.dispatch_forever();

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
    uint8_t pixels8lh[d6t8.N_PIXEL] = {0};
    // Temp sensor readings for 8 pixel thermal sensor
    if (cfg.has_tiretemp_1x8 && i2c_fail_count < 10) {
        auto read_result = d6t8.read();

        if (!read_result) {
            //printf("Read d6t8!\n: ");
            const double* px8 = d6t8.pixels_c();
            for (int i = 0; i < d6t8.N_PIXEL; i++) {
                pixels8lh[i] = (uint8_t)(px8[i]);
            }
        } else {
            printf("Failed reading d6t8 : %d\n", read_result);
            pixels8lh[0] = 66;
            i2c_fail_count++;
        }
    }

    // Tire temperature message
    CANMessage tpdo_tiretemp_msg(cfg.tpdo_tiretemp_id, pixels8lh, 8);
    can.write(tpdo_tiretemp_msg);
    last_sent_temp = canMsgTimer.elapsed_time().count();
    can.reset();
}

void sendCANtpdo() {
    uint16_t wheel_speed_raw = 0;
    uint16_t sus_travel_raw = 0;
    uint8_t px0 = 0;

    // // Side temp readings
    // if (ok1 && d6t1.read()) {
    //     // 1 pixel temp sensor, DATA_SIDE_TIRE_TEMP
    //     px0 = (uint8_t)d6t1.pixel_c(); // pixel temp
    // }

    // Wheel Speed Readings
    wheel_speed_raw = (int16_t)(wheelSensor.update() * 10); // scaled according to CAN.dbc values

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
    auto canResult = can.write(tpdo_msg);
    // printf("%d:: WRITE_ERR: %d\n", canMsgTimer.elapsed_time().count(), canResult);
    last_sent_tpdo = canMsgTimer.elapsed_time().count();

    //printf("CAN TX ERR CNT: %d\n", can.tderror());
}
