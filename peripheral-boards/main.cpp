#include "mbed.h"
#include "d6t-8lh.h"
#include "d6t-1a.h"
#include "config.h"
#include "wheel_speed.h"

static CAN can{PIN_CAN1_RX,PIN_CAN1_TX,CAN_FREQUENCY};
static WheelSpeed wheelsensor{PIN_WHEEL_SENSOR, TEETH_PER_REV, TIMEOUT_US};
static AnalogIn sus{PIN_SUSPENSION};
static I2C i2c{PIN_I2C2_SDA, PIN_I2C2_SCL};
static D6T8LH d6t8{i2c};
static D6T1A d6t1{i2c};
static CornerConfig cfg;
EventQueue queue = EventQueue{EVENTS_EVENT_SIZE*32};

int main()
{
    printf("main()\n");
    cfg = getCornerConfig(readCorner());
    d6t8.setup();
    d6t1.setup();

    queue.call_every(10ms, &sendCANtpdo);
    queue.call_every(10ms, &sendCANtemp);
    queue.dispatch_forever();

    return 0;
}

void sendCANtemp(){
    uint8_t pixels8lh[d6t8.N_PIXEL] = {0};
    //Temp sensor readings for 8 pixel thermal  sensor
    if (ok8 && d6t8.read()) 
    {
        const double* px8 = d6t8.pixels_c();
        for (int i=0; i<d6t8.N_PIXEL; i++) {
            pixels8lh[i] = (uint8_t)(px8[i]); //Not sure if I can cast like this
        }
    }

    //Tire temperature message
    CANMessage tpdo_tiretemp_msg(cfg.tpdo_tiretemp_id,pixels8lh,8);
    can.write(tpdo_tiretemp_msg);

}

void sendCANtpdo()
{
    uint16_t wheel_speed_raw = 0;
    uint16_t sus_travel_raw = 0;
    uint8_t px0 = 0;

    //Side temp readings
    if (ok1 && d6t1.read()) 
    {
        //1 pixel temp sensor, DATA_SIDE_TIRE_TEMP
        px0  = (uint8_t)d6t1.pixel_c(); //pixel temp
    }

    //Wheel Speed Readings
    wheelsensor.update();
    wheel_speed_raw = (int16_t)(wheelsensor.update()*10); //scaled according to CAN.dbc values

    //Suspension Travel Readings
    sus_travel_raw = ((1.0-sus.read()) * 5000);

    //MISSING STRAIN GAUGE READINGS
    uint8_t tpdo_data[] = {
        static_cast<uint8_t>(wheel_speed_raw & 0xFF),
        static_cast<uint8_t>((wheel_speed_raw & 0xFF00) >> 8),
        static_cast<uint8_t>(sus_travel_raw & 0xFF), 
        static_cast<uint8_t>((sus_travel_raw & 0xFF00) >> 8),
        0x00,
        0x00,
        px0,
    };

    //tpdo message, tbh I don't really know what tpdo means
    CANMessage tpdo_msg(cfg.tpdo_data_id, tpdo_data,7);
    can.write(tpdo_msg);
}
