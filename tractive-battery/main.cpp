
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

constexpr bool eMeterPresent = false;

EventQueue queue(5 * EVENTS_EVENT_SIZE);

BMS BMSInstance;
CanGenerator cGen(&BMSInstance);

OneWire TS1W = OneWire{PB_14};
DigitalOut TS1W_PU_Control = DigitalOut(PB_15);
// clang-format off
DS18B20 temp_a {TS1W, 0x860000112ffda728};
DS18B20 temp_b {TS1W, 0x520000112fffdd28};
DS18B20 temp_c {TS1W, 0x7400001130aabd28};
DS18B20 temp_d {TS1W, 0x3e00001111126d28};
DS18B20 temp_e {TS1W, 0x0e0000111131fc28};
// clang-format on
DS18B20 trayTempSensors[NUM_TRAY_TEMP_SENSORS] = {temp_a, temp_b, temp_c, temp_d, temp_e};
uint8_t trayTemps[NUM_TRAY_TEMP_SENSORS];

int main() {
    for (uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS; i++) {
        trayTempSensors[i].start_conversion(true); // assume no e meter here CHANGE LATER
        ThisThread::sleep_for(3ms);
        uint8_t trayTemp = trayTempSensors[i].retrieve_conversion();
        trayTemps[i] = trayTemp;
    }

    // One wire section....
    // We are using the ds18b20 sensors on the 1 wire bus, according to
    // https://www.analog.com/en/resources/technical-articles/how-to-power-the-extended-features-of-1wire-devices.html
    // these sensors need a little extra power for temperature conversions.... when the emeter is
    // connected to the car it is able to provide this power when the e meter is not connected to
    // the car, a pmos pull up transistor is used to provide this extra power (I think).
    if (!eMeterPresent) {
        TS1W_PU_Control = 1;
    } else {
        TS1W_PU_Control = 0;
    }

    // TEMPORARY: searching for 1 wire sensors......
    // should print out to serial the address of any one wire bus temp sensor.
    debug_search_for_ds18b20_address(TS1W);

    printf("Initialization complete\n");

    queue.call_every(2ms, &BMSInstance, &BMS::controller);
    queue.call_every(1000ms, &cGen, &CanGenerator::BuildAndSendMessages);
    queue.dispatch_forever();

    return 0;
}
