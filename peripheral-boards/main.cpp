#include <thread>

#include "mbed.h"
#include "d6t-8lh.h"
#include "d6t-1a.h"
#include "config.h"
#include "wheel_speed.h"

uint16_t wheel_speed = 0;
uint16_t sus_travel = 0;

CAN *can(PIN_CAN1_RX,PIN_CAN1_TX,CAN_FREQUENCY);
WheelSpeed wheelsensor(PIN_WHEEL_SENSOR, TEETH, 5ms);
DigitalIn dip1(PIN_DIP_1,PullDown);
DigitalIn dip2(PIN_DIP_2,PullDown);
AnalogIn sus(PIN_SUSPENSION);
D6T8LH d6t8(i2c);
D6T1A  d6t1(i2c);
uint8_t pixels8lh[d6t8.N_PIXEL] = {0};
I2C i2c(I2C_SDA, I2C_SCL);
bool ok8 = d6t8.setup();
bool ok1 = d6t1.setup();

cornerConfig cfg;
Thread eventThread;
EventQueue queue = EventQueue(EVENTS_EVENT_SIZE*32);

int main()
{
    printf("main()\n");
    cfg = getCornerConfig(readCorner());
    queue.call_every(100ms, &sendCAN);
    printf("Starting main loop\n");
    
    // Main loop
    while (true) {

        //Temp sensor readings
        if (ok8 && d6t8.read()) 
        {
            //8 Thermal pixel temperature
            const double* px8 = d6t8.pixels_c(); 
            for (int i=0; i<d6t8.N_PIXEL; i++) {
                pixels8lh[i] = (uint8_t)(px8[i]);
            }
        }

        if (ok1 && d6t1.read()) 
        {
            //NOT SURE WHAT TO DO HERE
            //1 pixel temp sensor
            double ptat1 = d6t1.ptat_c(); //reference temp (don't think its needed)
            double px0  = d6t1.pixel_c(); //pixel temp
            //NOT SURE HOW TO SEND THIS MESSAGE
        }

        //Wheel Speed Readings
        wheelsensor.update();
        float rpm = wheelsensor.getRPM();
        wheel_speed = rpm * TIRE_CIRCUMFERENCE; //I have no idea what units speed is measured in mph?

        //Suspension Travel Readings
        sus_travel = sus.read() * 5000; 
        //FS-3 got a 10 bit adc reading, converted it to 16, inverted and normalized, then multipled by 5000
        // const uint16_t sus_travel_voltage = ADC1_GetConversion(ADC_MUXPOS_AIN2_gc) * pow(2, 16 - 10);
        // const uint16_t sus_travel = (pow(2, 16) - sus_travel_voltage) / pow(2, 16) * 5000;
        queue.dispatch_once();
    }
    return 0;
}

void sendCAN(){
    //Tire temperature message
    CANMessage tpdo_tiretemp_msg(cfg.tpdo_tiretemp_id,pixels8lh,8);
    can->write(tpdo_tiretemp_msg);
    ThisThread::sleep_for(1ms);
    //Wheel and Suspension travel message
    uint8_t tpdo_data[] = {
        wheel_speed & 0xFF,
        (wheel_speed & 0xFF00) >> 8,
        sus_travel & 0xFF, 
        (sus_travel & 0xFF00) >> 8,
        0x00,
        0x00,
        0x00,
    };
    CANMessage tpdo_msg(cfg.tpdo_data_id, tpdo_data,7);
    can->write(tpdo_msg);
    ThisThread::sleep_for(1ms);
}
Corner readCorner()
{
    //dip1 is first bit, dip2 is next bit
    int val = (!dip2.read() << 1) | !dip1.read();
    switch(val) 
    {
        case 0: return FR;
        case 1: return FL;
        case 2: return BR;
        case 3: return BL;
        default: return FR;
    }
}
cornerConfig getCornerConfig(Corner pos)
{
    switch(pos) {
        case FR:
            return {0x1A3, 0x2A2, false, false};

        case FL:
            return {0x1A2, 0x2A1, true, false};

        case BR:
            return {0x1A5, 0x2A4, false, false};

        case BL:
            return {0x1A4, 0x2A3, true, false};

        default:
            return {0x1A3, 0x2A2, false, false}; 
            //Defaulting to FR, Not sure what to do for an error
            //FS-3 #error "WHEEL_POSITION must be one of BR/BL/FR/FL!"
    }
}