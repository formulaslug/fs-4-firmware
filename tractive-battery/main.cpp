#include "AnalogIn.h"
#include "PeripheralNames.h"
#include "PinNamesTypes.h"
#include "can.h"


inline constexpr uint16_t analogInThreshold = 3.3*1000;
inline constexpr uint16_t glvVoltageScaling = 3.3 * 1000 * 3.94 * 2;

Mutex TelemetryLock;

CAN canPowertrain = CAN(PA_11, PA_12, 500000);
CAN canDatatrain = CAN(PB_12, PB_13, 1000000);

AnalogIn GLV_Voltage = AnalogIn(PA_7);
// 1 = charging, 0 = not charging
AnalogIn Charge_State_Filtered = AnalogIn(PC_2);
// Status of the shutdown circuit before BMS & IMD
AnalogIn Shutdown_In_3V3_Filtered = AnalogIn(PA_0);
// Status of the shutdown circuit after BMS & IMD
AnalogIn Shutdown_Out_3V3_Filtered = AnalogIn(PA_1);
// // Status of the shutdown circuit after BMS & IMD & HV Interlock & TSMS
AnalogIn Shutdown_Final_3V3_Filtered = AnalogIn(PA_6);
// DigitalIn Shutdown_Final_3V3_Filtered = DigitalIn(PA_6);
// Note: IMD_Fault_3V3 is actually fault-low, and should be called nIMD_Fault_3V3
AnalogIn nIMD_Fault_3V3 = AnalogIn(PC_4);

DigitalOut TS_READY = DigitalOut(PC_9);
PwmOut Fan_PWM = PwmOut(PC_8);

bool prechargeDone = false;
bool precharging = false;
// This MUST be set to 0 by default, otherwise precharge will not occur at all
// and there will be a massive current spike that blows the pack fuse. It is
// IMPERATIVE that this only becomes 1 once precharge is _finished_.
DigitalOut nPrechargeControl = DigitalOut(PB_0, 0);

float fanPwmDuty = 0.0f;

EventQueue queue(64 * EVENTS_EVENT_SIZE);

Thread bmsControllerThread{osPriorityHigh};
EventQueue bmsEventQueue(16 * EVENTS_EVENT_SIZE);

BMS bms(canPowertrain, ((Charge_State_Filtered.read() * 3.3 * 1000) > 700), TelemetryLock);

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
// void updateSoc();
void sendCanMessages(CAN &can_train);
void sampleShutdownFinal();

int main() {
    TS_READY = 0;
    printf("%f v\n", Charge_State_Filtered.read() * 3.3);

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
        canDatatrain.filter(0x190, 0x1ff);
    } else {
        canPowertrain.filter(0x682, 0xfff);
        canDatatrain.filter(0x682, 0xfff);
    }
    //canPowertrain.attach([]() { queue.call(&processCanRx); }, CAN::IrqType::RxIrq);

    // Start updating precharge, and also again whenever shutdown opens
    queue.call_every(10ms, &updatePrecharge);
    queue.call_every(5ms, &processCanRx);

    // TODO: Verify and uncoment
    // queue.call_every(500ms, sampleShutdownFinal);

    queue.call_every(500ms, controlFans);

    bmsEventQueue.call_every(200ms, &bms, &BMS::controller);
    bmsControllerThread.start(callback(&bmsEventQueue, &EventQueue::dispatch_forever));

    queue.call_every(100ms, []() { sendCanMessages(canPowertrain); });
    queue.call_every(200ms, []() { sendCanMessages(canDatatrain); });

    queue.dispatch_forever();

    return 0;
}

// void updateSoc(){
//     Data.socEstimate =
//     socEstimator.update((BMSInstance.packCurrentAmpsOutput-BMSInstance.packCurrentAmpsInput),
//     BMSInstance.packVoltageMv);
// }

void controlFans() {
    if (prechargeDone) {
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
                // printf("dcBusVoltageMv (charger): %d\n", dcBusVoltageMv);
            }
            break;
        }
        case BMS::ACTIVE: {
            switch (msg.id) {
            case 0x682: // SME_TPDO_Temperature
                dcBusVoltageMv = (msg.data[2] | (msg.data[3] << 8))*100;
            }
            break;
        }
        case BMS::FAULT:
            break;
        }
    }
}

bool shutdownClosed() { return ((Shutdown_Final_3V3_Filtered.read()*analogInThreshold)>700); }

uint16_t glvVoltageMv() { return (uint16_t)(GLV_Voltage.read() * glvVoltageScaling); }

// Intented to be called repeatedly during start and until completion of
// precharge.
void updatePrecharge() {
    // printf("dcBusVoltageMv %d mv, packVoltageMv %d mv\n", dcBusVoltageMv, bms.packVoltageMv);
    if (dcBusVoltageMv > 0.9 * bms.packVoltageMv) {
        // printf("Precharge Finished!\n");
        nPrechargeControl = 1;
        precharging = false;
        if (shutdownClosed()) {
            prechargeDone = true;
            TS_READY = 1;
        } else {
            prechargeDone = false;
            TS_READY = 0;
        }
    }
    if (dcBusVoltageMv < 0.2 * bms.packVoltageMv) {
        // printf("Precharge Starting!\n");
        nPrechargeControl = 0;
        precharging = true;
        prechargeDone = false;
        TS_READY = 0;
    }

}

void sendCanMessages(CAN &can_train) {
    // TelemetryLock.lock();
    CANMessage msg;

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        msg = CanGenerator::BuildVoltageMessage(bms, i);
        can_train.write(msg);
        msg = CanGenerator::BuildTempMessage(bms, i, true);
        can_train.write(msg);
        msg = CanGenerator::BuildTempMessage(bms, i, false);
        can_train.write(msg);
        ThisThread::sleep_for(2ms);
    }
    msg = CanGenerator::BuildPowerMessage(bms, 0);
    can_train.write(msg);
    msg = CanGenerator::BuildStatusMessage(
        bms,
        ((nIMD_Fault_3V3.read()*analogInThreshold)<700),
        ((Shutdown_Final_3V3_Filtered.read()*analogInThreshold)>700),
        ((Shutdown_In_3V3_Filtered.read()*analogInThreshold)>700),
        ((Shutdown_Out_3V3_Filtered.read()*analogInThreshold)>700),
        precharging,
        prechargeDone,
        glvVoltageMv(),
        fanPwmDuty
    );
    can_train.write(msg);

    //printf("\033[2J");
    //printf("IMD fault voltage (normally high): %f\n", nIMD_Fault_3V3.read()*3.3);
    //printf("CAN    RTRN: %d, TDERRCNT: %d, RDERRCNT: %d\n", canPowertrain.write(msg), canPowertrain.tderror(), canPowertrain.rderror());

    can_train.reset();
    // TelemetryLock.unlock();
}
