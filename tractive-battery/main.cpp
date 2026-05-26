
#include "mbed.h"
// #include "BMS.h"
#include "tempCan.h"

/*
TODO items:
        - TELEMETRY
        - calibrate and understand current sensing ...
        - test precharge go over logic make sure everything at least makes sense
        - make sure our info being sent to can is kosher (can use a spare can transciever??)

*/
// need to initialize everything on startup - assume everything is okay at first

CAN CAN_POWERTRAIN = CAN(PA_11, PA_12, 500000);

// assume 1 for charging 0 for not charging
DigitalIn Charge_State_Filtered = DigitalIn(PC_2);

// Status of the shutdown circuit before BMS & IMD
DigitalIn Shutdown_In_3V3_Filtered = DigitalIn(PA_0);
// Status of the shutdown circuit after BMS & IMD
DigitalIn Shutdown_Out_3V3_Filtered = DigitalIn(PA_1);
InterruptIn Shutdown_Final_3V3_Filtered = InterruptIn(PA_6);

constexpr float PRECHARGE_TIMEOUT = 3.0f;
DigitalOut nPrechargeControl = DigitalOut(PB_0);
Timer prechargeTimer;
int prechargeUpdateEventId;

constexpr bool eMeterPresent = false;

EventQueue queue(5 * EVENTS_EVENT_SIZE);

BMS BMSInstance(CAN_POWERTRAIN, Charge_State_Filtered.read());
CanGenerator cGen(BMSInstance, CAN_POWERTRAIN);

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

uint32_t dcBusVoltageMv;

struct TelemetryInfo {
    bool bmsFaultStatus;
    bool imdStatus;
    bool shutdownIn;
    bool shutdownOut;
    bool shutdownFinal;
    bool preChargeActive;
    bool prechargeDone;
    bool charging;

    bool balanceStat;
    bool cellTooLow;
    bool cellTooHigh;
    bool tempTooLow;
    bool tempTooHigh;
    bool tempTooHighCRG;
    uint8_t faultModIndex;
    uint8_t faultSenseIndex;
    uint8_t battStatFaultIndex; // this is the cell fault num
    uint16_t glvVoltage;
    uint8_t pwmFanstat;
} Data;

void processCanRx();
bool prechargeAllowed();
bool prechargeComplete();
bool shutdownClosed();
void updatePrecharge();

// enum precharge_state { PRECHARGE_IDLE, PRECHARGE_ACTIVE, PRECHARGE_FAULT, PRECHARGE_COMPLETE };
// precharge_state prechargeState = PRECHARGE_IDLE;

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
    // debug_search_for_ds18b20_address(TS1W);

    printf("Initialization complete\n");

    CAN_POWERTRAIN.attach([]() { queue.call(&processCanRx); }, CAN::IrqType::RxIrq);

    Shutdown_Final_3V3_Filtered.fall([&]() {
        Data.prechargeDone = false;
        prechargeUpdateEventId = queue.call_every(2ms, &updatePrecharge);
    });

    queue.call_every(2ms, &BMSInstance, &BMS::controller);
    // queue.call_every(1000ms, &cGen, &CanGenerator::BuildAndSendMessages);

    while (Data.prechargeDone) {
        updatePrecharge();
    }

    queue.dispatch_forever();

    return 0;
}

void processCanRx() {
    CANMessage msg;
    while (CAN_POWERTRAIN.read(msg)) {
        switch (BMSInstance.currentState) {
        case BMS::CHARGING: {
            switch (msg.id) {
            case 0x190: // charge status from charger, 180 + node ID (10)
                dcBusVoltageMv =
                    (msg.data[2] | (msg.data[3] << 8) | (msg.data[4] << 16) | (msg.data[5] << 24))
                    / 100.0;
            }
            break;
        }
        case BMS::ACTIVE: {
            switch (msg.id) {
            case 0x682: // SME_TPDO_Temperature
                dcBusVoltageMv = (msg.data[2] | (msg.data[3] << 8));
            }
            break;
        }
        case BMS::FAULT:
          break;
        }
    }
}

bool shutdownClosed() { return Shutdown_Final_3V3_Filtered.read(); }

bool glvOk() {
    float glvVoltage = (uint16_t)(BMSInstance.GLV_Voltage.read() * 3.3 * 21.9 / 3.9 * 1000);
    //  BMSInstance->bms_stat_message.glv_voltage = glvVoltage;
    return (
        glvVoltage > 11.0f && glvVoltage < 15.0f
    ); // place holder for 11 to 15 volts? we should find out the limit
}

bool imdOk() { return BMSInstance.IMD_Fault_3V3.read(); }

bool prechargeAllowed() {
    return shutdownClosed()
           && BMSInstance.currentState != BMS::bms_state::FAULT
           && glvOk()
           && imdOk()
           && (dcBusVoltageMv < 0.2f * BMSInstance.packVoltageMv)
           && BMSInstance.packVoltageMv > 60; // sanity check
}

bool prechargeComplete() { return dcBusVoltageMv >= 0.9f * BMSInstance.packVoltageMv; }

// Intented to be called repeatedly during start and until completion of
// precharge.
void updatePrecharge() {
    if (prechargeAllowed() && !prechargeComplete()) {
        // Start precharge
        nPrechargeControl = 0; // low during precharge!

        Data.preChargeActive = true;
        prechargeTimer.reset();
        prechargeTimer.start();
    } else if (Data.preChargeActive && prechargeComplete()) {
        // Stop precharge, success
        nPrechargeControl = 1;

        Data.prechargeDone = true;
        Data.preChargeActive = false;
        prechargeTimer.stop();
        prechargeTimer.reset();

        // stop updating precharge, shutdown should be closed now
        queue.cancel(prechargeUpdateEventId);

    } else if (Data.preChargeActive
               && (!prechargeAllowed() || prechargeTimer.read() > PRECHARGE_TIMEOUT))
    {
        // Stop precharge, failure
        nPrechargeControl = 1; // high to stop precharge

        Data.preChargeActive = false;
        Data.prechargeDone = false;
        printf("Precharge failed!\n");

        prechargeTimer.stop();
        prechargeTimer.reset();

        // stop updating precharge; start updating again in 5s.
        queue.cancel(prechargeUpdateEventId);
        queue.call_in(5s, [&]() {
            prechargeUpdateEventId = queue.call_every(2ms, &updatePrecharge);
        });
    }
}
