#include "mbed.h"
// #include "BMS.h"
#include "can.h"

CAN canPowertrain = CAN(PA_11, PA_12, 500000);

AnalogIn GLV_Voltage = AnalogIn(PA_7);
// 1 = charging, 0 = not charging
DigitalIn Charge_State_Filtered = DigitalIn(PC_2);
// Status of the shutdown circuit before BMS & IMD
DigitalIn Shutdown_In_3V3_Filtered = DigitalIn(PA_0);
// Status of the shutdown circuit after BMS & IMD
DigitalIn Shutdown_Out_3V3_Filtered = DigitalIn(PA_1);
// // Status of the shutdown circuit after BMS & IMD & HV Interlock & TSMS
InterruptIn Shutdown_Final_3V3_Filtered_irq = InterruptIn(PA_6);
DigitalIn Shutdown_Final_3V3_Filtered = DigitalIn(PA_6);
// Note: IMD_Fault_3V3 is actually fault-low, and should be called nIMD_Fault_3V3
DigitalIn nIMD_Fault_3V3 = DigitalIn(PC_4);

DigitalOut TS_READY = DigitalOut(PC_9);
PwmOut Fan_PWM = PwmOut(PC_8);

bool prechargeDone = false;
bool precharging = false;
constexpr float PRECHARGE_TIMEOUT_S = 3.0f;
DigitalOut nPrechargeControl = DigitalOut(PB_0);
Timer prechargeTimer;
int prechargeUpdateEventId;

float fanPwmDuty = 0.0f;

EventQueue queue(64 * EVENTS_EVENT_SIZE);

Thread bmsControllerThread;
EventQueue bmsEventQueue(16 * EVENTS_EVENT_SIZE);

BMS bms(canPowertrain, Charge_State_Filtered.read());

// the following is test soc code to get soc integrated into tbb firmware

// KalmanSOC socEstimator(100, 100, 1);

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

void processCanRx();
bool prechargeAllowed();
bool prechargeComplete();
bool shutdownClosed();
void updatePrecharge();
void controlFans();
void updateTelemetry();
void updateSoc();
void sendCanMessages();

// enum precharge_state { PRECHARGE_IDLE, PRECHARGE_ACTIVE, PRECHARGE_FAULT, PRECHARGE_COMPLETE };
// precharge_state prechargeState = PRECHARGE_IDLE;

int main() {
    TS_READY = 0;

    // TODO: uncommenct once we have tray temp sensors installed
    // for (uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS; i++) {
    //     trayTempSensors[i].start_conversion(true); // assume no e meter here CHANGE LATER
    //     ThisThread::sleep_for(3ms);
    //     uint8_t trayTemp = trayTempSensors[i].retrieve_conversion();
    //     trayTemps[i] = trayTemp;
    // }
    // We are using the ds18b20 sensors on the 1 wire bus, according to
    // https://www.analog.com/en/resources/technical-articles/how-to-power-the-extended-features-of-1wire-devices.html
    // these sensors need a little extra power for temperature conversions.... when the emeter is
    // connected to the car it is able to provide this power when the e meter is not connected to
    // the car, a pmos pull up transistor is used to provide this extra power (I think).
    // if (!eMeterPresent) {
    //     TS1W_PU_Control = 1;
    // } else {
    //     TS1W_PU_Control = 0;
    // }
    TS1W_PU_Control = 0;
    // TEMPORARY: searching for 1 wire sensors......
    // should print out to serial the address of any one wire bus temp sensor.
    // while (1) {
    //     debug_search_for_ds18b20_address(TS1W);
    // }

    switch (bms.currentState) {
    case BMS::ACTIVE:
        printf("TBB main(): Current State: ACTIVE \n");
        break;
    case BMS::CHARGING:
        printf("TBB main(): Current State: CHARGING \n");
        break;
    case BMS::FAULT:
        printf("TBB main(): Current State: FAULT \n");
        break;
    }

    if (bms.currentState == BMS::CHARGING) {
        canPowertrain.filter(0x190, 0x1ff);
    } else {
    }
    canPowertrain.attach([]() { queue.call(&processCanRx); }, CAN::IrqType::RxIrq);

    // Start updating precharge, and also again whenever shutdown opens
    queue.call_every(10ms, &updatePrecharge);
    // TODO: instead of irq, use polling on an ADC with hysteresis (fall is <0.3, rise is >3.0)
    Shutdown_Final_3V3_Filtered_irq.fall([&]() {
        queue.call(printf, "Shutdown_Final: Falling Edge!\n");
        prechargeDone = false;
        if (prechargeTimer.elapsed_time().count() == 0) {
            prechargeUpdateEventId = queue.call_every(2ms, &updatePrecharge);
        }
    });

    // queue.call_every(200ms, controlFans);

    bmsEventQueue.call_every(5ms, &bms, &BMS::controller);
    bmsControllerThread.start(callback(&bmsEventQueue, &EventQueue::dispatch_forever));

    queue.call_every(100ms, sendCanMessages);

    queue.dispatch_forever();

    return 0;
}

// void updateSoc(){
//     Data.socEstimate =
//     socEstimator.update((BMSInstance.packCurrentAmpsOutput-BMSInstance.packCurrentAmpsInput),
//     BMSInstance.packVoltageMv);
// }

void controlFans() {
    if (!precharging) {
        // Linear scaling: 20% at ~20°C, 100% at ~50°C
        // Formula: (2.6667 * temp) - 33.3333, clamped to [20, 100]
        int raw_percent = (int)((2.6667f * bms.maxCellTemp) - 33.3333f);
        uint8_t fan_percent = (uint8_t)std::clamp(raw_percent, 20, 100);
        fanPwmDuty = fan_percent / 100.0f;
        Fan_PWM.write(fanPwmDuty);
    } else {
        // Keep fans off until precharge is complete
        fanPwmDuty = 0;
        Fan_PWM.write(fanPwmDuty);
    }
}

void processCanRx() {
    CANMessage msg;
    while (canPowertrain.read(msg)) {
        switch (bms.currentState) {
        case BMS::CHARGING: {
            switch (msg.id) {
            case 0x190: // charge status from charger, 180 + node ID (10)
                dcBusVoltageMv =
                    (msg.data[2] | (msg.data[3] << 8) | (msg.data[4] << 16) | (msg.data[5] << 24));
                printf("dcBusVoltageMv (charger): %d\n", dcBusVoltageMv);
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

bool shutdownClosed() { return Shutdown_Final_3V3_Filtered_irq.read(); }

bool glvVoltageV() { return (uint16_t)(GLV_Voltage.read() * 3.3 / (6.8 / (6.8 + 20))); }

bool imdOk() { return true || nIMD_Fault_3V3.read(); }

bool prechargeAllowed() {
    return !shutdownClosed()
           && bms.currentState != BMS::bms_state::FAULT
           && glvVoltageV() > 11.0f
           && glvVoltageV() < 15.0f
           && imdOk()
           && (dcBusVoltageMv < 0.2f * bms.packVoltageMv)
           && bms.packVoltageMv > 60; // sanity check
}

bool prechargeComplete() { return dcBusVoltageMv >= 0.9f * bms.packVoltageMv; }

// Intented to be called repeatedly during start and until completion of
// precharge.
void updatePrecharge() {
    if (!precharging && prechargeAllowed() && !prechargeComplete()) {
        // Start precharge
        nPrechargeControl = 0; // low during precharge!

        precharging = true;
        prechargeDone = false;
        prechargeTimer.reset();
        prechargeTimer.start();
    } else if (precharging && prechargeComplete()) {
        // Stop precharge, success
        nPrechargeControl = 1;

        precharging = false;
        prechargeDone = true;
        TS_READY = 1;
        prechargeTimer.stop();
        prechargeTimer.reset();

        // stop updating precharge, shutdown should be closed now
        queue.cancel(prechargeUpdateEventId);

    } else if (precharging && (!prechargeAllowed() || prechargeTimer.read() > PRECHARGE_TIMEOUT_S))
    {
        // Stop precharge, failure
        nPrechargeControl = 1; // high to stop precharge

        precharging = false;
        prechargeDone = false;
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

void sendCanMessages() {
    CANMessage msg;

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        msg = CanGenerator::BuildVoltageMessage(bms, i);
        canPowertrain.write(msg);
        msg = CanGenerator::BuildTempMessage(bms, i, true);
        canPowertrain.write(msg);
        msg = CanGenerator::BuildTempMessage(bms, i, false);
        canPowertrain.write(msg);
        ThisThread::sleep_for(2ms);
        msg = CanGenerator::BuildPowerMessage(bms, 0);
        canPowertrain.write(msg);
        ThisThread::sleep_for(2ms);
    }
    msg = CanGenerator::BuildStatusMessage(
        bms,
        !nIMD_Fault_3V3.read(),
        Shutdown_Final_3V3_Filtered.read(),
        Shutdown_In_3V3_Filtered.read(),
        Shutdown_Out_3V3_Filtered.read(),
        precharging,
        prechargeDone,
        glvVoltageV(),
        fanPwmDuty
    );

    canPowertrain.write(msg);
}
