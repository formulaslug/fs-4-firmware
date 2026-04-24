
#include "mbed.h"
// #include "BMS.h"
#include "prechargeLogic.h"
#include "tempCan.h"

/*
TODO items:
        - TELEMETRY
        - calibrate and understand current sensing ...
        - test precharge go over logic make sure everything at least makes sense
        - make sure our info being sent to can is kosher (can use a spare can transciever??)

*/
// need to initialize everything on startup - assume everything is okay at first

bool eMeterPresent = false;

EventQueue queue(5 * EVENTS_EVENT_SIZE);
BMS BMSInstance;
CanGenerator cGen(&BMSInstance);

std::vector<DS18B20> trayTempSensors;

uint8_t trayTemps[NUM_TRAY_TEMP_SENSORS];

// BMSInstance = BMS();
int main() {

    // there should also be the startup checks for the ADCS on all the LTC6810s here. not a priority
    // but nice to havce

    BMSInstance.VCP_UART.set_baud(115200); // intializing serial interface

    for (uint i = 0; i < NUM_BATTERY_MODULES; i++) {
        BMSInstance.chips.push_back(
            LTC6810(BMSInstance.ltcBusInterface, i)
        ); // initializing ltc chip objects
    }

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) { // initlaize the tmp1075 handlers
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            LTC6810::TMP1075_Handle_t sens;
            sens.i2c_address = TMP1075_ADDRESSES[j];
            sens.temp_reg = 0x00; // it gives out 0 degrees until the first conversion so
            BMSInstance.sensors[i][j] = sens;
        }
    }

    for (uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS; i++) {
        trayTempSensors[i].start_conversion(true); // assume no e meter here CHANGE LATER
        ThisThread::sleep_for(3ms);
        uint8_t trayTemp = trayTempSensors[i].retrieve_conversion();
        trayTemps[i] = trayTemp;
    }

    // One wire section....
    /*
    we are using the ds18b20 sensors on the 1 wire bus, according to
    https://www.analog.com/en/resources/technical-articles/how-to-power-the-extended-features-of-1wire-devices.html
    these sensors need a little extra power for temperature conversions.... when the emeter is
    connected to the car it is able to provide this power when the e meter is not connected to the
    car, a pmos pull up transistor is used to provide this extra power (I think).

    */

    if (!eMeterPresent) {
        BMSInstance.TS1W_PU_Control = 1;
    } else {
        BMSInstance.TS1W_PU_Control = 0;
    }

    // TEMPORARY: searching for 1 wire sensors......

    // should print out to serial the address of any one wire bus temp sensor is it the same as fs3?
    // idk
    debug_search_for_ds18b20_address(BMSInstance.TS1W);

    printf("Initialization complete\n");

    queue.call_every(2ms, &BMSInstance, &BMS::controller);
    queue.call_every(1000ms, &cGen, &CanGenerator::BuildAndSendMessages);
    queue.dispatch_forever();

    return 0;
}