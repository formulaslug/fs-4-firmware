#include "config.hpp"
#include "d6t-1a.h"
#include "d6t-8lh.h"
#include "mbed.h"
#include "wheel_speed.hpp"
// #include "strain_gauge_235sl.h" // TODO: Uncomment when StrainGuage PR is merged

CAN can{PIN_CAN1_RX, PIN_CAN1_TX, CAN_FREQUENCY};
WheelSpeed wheelsensor{PIN_WHEEL_SENSOR, TEETH_PER_REV};
DigitalIn wheelPin{PIN_WHEEL_SENSOR};
AnalogIn sus{PIN_SUSPENSION};
I2C i2c{PIN_I2C2_SDA, PIN_I2C2_SCL};
D6T8LH d6t8{i2c};
D6T1A d6t1{i2c};
//StrainGauge235SL sg{PIN_STRAIN}; // TODO: Uncomment when StrainGuage PR is merged
//Suggestion: Set the output to also include ticks since the last can message was sent
// Not really sure what to do with this output yet
Timer canMsgTimer;
CornerConfig cfg;
EventQueue queue = EventQueue{EVENTS_EVENT_SIZE * 32};
uint64_t last_sent_temp = 0;
uint64_t last_sent_tpdo = 0;

int main() {
    printf("main()\n");
    wheelPin.mode(PullUp);
    cfg = getCornerConfig(readCorner());
    //printf("readCorner(): %d\n", static_cast<int>(readCorner()));
    d6t8.setup();
    d6t1.setup();

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
    queue.call_every(10ms, &sendCANtpdo);
    queue.call_every(10ms, &sendCANtemp);
    queue.dispatch_forever();
    
    return 0;
}

void sendCANtemp() {
    uint8_t pixels8lh[d6t8.N_PIXEL] = {0};
    // Temp sensor readings for 8 pixel thermal  sensor
    if (ok8 && d6t8.read()) {
        const double* px8 = d6t8.pixels_c();
        for (int i = 0; i < d6t8.N_PIXEL; i++) {
            pixels8lh[i] = (uint8_t)(px8[i]); // Not sure if I can cast like this
        }
    }

    // Tire temperature message
    CANMessage tpdo_tiretemp_msg(cfg.tpdo_tiretemp_id, pixels8lh, 8);
    can.write(tpdo_tiretemp_msg);
    uint64_t current_time = canMsgTimer.elapsed_time().count();
    printf("Time since last temp: %d", current_time - last_sent_temp);
    last_sent_temp = current_time;
}

void sendCANtpdo() {
    uint16_t wheel_speed_raw = 0;
    uint16_t sus_travel_raw = 0;
    uint8_t px0 = 0;

    // Side temp readings (I'm also not sure if I need to clamp any of these readings)
    if (ok1 && d6t1.read()) {
        // 1 pixel temp sensor, DATA_SIDE_TIRE_TEMP
        px0 = (uint8_t)d6t1.pixel_c(); // pixel temp
    }

    // Wheel Speed Readings
    wheel_speed_raw = (int16_t)(wheelsensor.update() * 10); // scaled according to CAN.dbc values
    //printf("WheelSpeed(CAN): %d \n", wheel_speed_raw);

    // Suspension Travel Readings
    sus_travel_raw = ((1.0 - sus.read()) * 5000);

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
    can.write(tpdo_msg);
    uint64_t current_time = canMsgTimer.elapsed_time().count();
    printf("Time since last temp: %d", current_time - last_sent_tpdo);
    last_sent_tpdo = current_time;
}
