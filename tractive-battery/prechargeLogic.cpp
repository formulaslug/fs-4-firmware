#include "prechargeLogic.h"
// #include "BMS.h"
#include "mbed.h"
// #include "BMSFaultDetection.h"

/*
I think some major rewrites are needed here, i dont think the logic fully holds up
*/

bool precharging = false;
bool prechargeDone = false;

float dcBusVoltage = 0.0;
float packVoltage = 400.0; // shouldnt be set just here....

precharge_state prechargeState = PRECHARGE_IDLE;
Timer prechargeTimer;
const float PRECHARGE_TIMEOUT = 3.0f; // placeholder timeout value in seconds

// BMS BMSInstance;
float readDcBusVoltage() {
    BMSInstance.CAN_POWERTRAIN.read(BMSInstance.msg);
    uint32_t id = BMSInstance.msg.id;
    unsigned char* data = BMSInstance.msg.data;
    if (!BMSInstance.Charge_State_Filtered.read()) {
        switch (id) { // copied over from fs3
        case 0x682:   // temperature message from MC
            dcBusVoltage = (data[2] | (data[3] << 8));
            break;
        default:
            break;
        }
    }
    return dcBusVoltage; // this is not correct has to be read from can, but we can use fs3
                         // implementation for now
}

bool glvOk() {
    float glvVoltage = (uint16_t)(BMSInstance.GLV_Voltage.read() * 3.3 * 21.9 / 3.9 * 1000);
    // CurrentBMSStatus.glv_voltage = (uint16_t)(glv * 1000);
    //  BMSInstance.bms_stat_message.glv_voltage = glvVoltage;
    return (
        glvVoltage > 11.0f && glvVoltage < 15.0f
    ); // place holder for 11 to 15 volts? we should find out the limit
}

bool isBmsFaultActive() { return BMSInstance.currentState; }

bool shutdownClosed() {
    return BMSInstance.Shutdown_Measure.read();
    // we also need to add safety checks here to make sure we dont precharge when the shutdown
    // circuit is open anywhere else
}

bool imdOk() {
    // return imdFaultPin.read();
    // i think this should probably be in BMSFaultDetection, but for now
    return BMSInstance.IMD_Fault_3V3.read();
}

bool preChargeAllowed() {
    return shutdownClosed()
           && imdOk()
           && !isBmsFaultActive()
           && glvOk()
           && dcBusVoltage < 0.9f * packVoltage;
}
bool prechargeComplete(float packVoltage) {
    return dcBusVoltage >= 0.9f * packVoltage; // i think this is good i wanna double check tho
}

void updatePrecharge() {
    switch (prechargeState) {
    case PRECHARGE_IDLE:
        precharging = false;
        if (BMSInstance.currentState
            == BMSInstance.ACTIVE) { // added by Ethan going to be removed later
                                     // CurrentBMSStatus.prechargeDone = false;
            /*
                I dont think the logic is correct here, does it just automatically precharge if the
               battery packs are okay and its not already precharging? theres no checks to see if we
               have a good dcBusVoltage in precharge allowed.....
            */
            if (preChargeAllowed() && packVoltage >= MIN_PACK_MV) {
                BMSInstance.nPrechargeControl = 0; // outputs low during precharge!
                // prechargeRelay = 1;
                precharging = true;
                // CurrentBMSStatus.precharging = true;
                // CurrentBMSStatus.prechargeDone = false;
                BMSInstance.currentState = BMSInstance.PRECHARGING;
                prechargeTimer.reset();
                prechargeTimer.start();
                prechargeState = PRECHARGE_ACTIVE;
            }
            break;
        }

    case PRECHARGE_ACTIVE:
        if (!preChargeAllowed()) {
            BMSInstance.nPrechargeControl = 1; // outputs hight to stop precharge
            // prechargeRelay = 0;
            precharging = false;
            prechargeDone = false;
            // might need to add some telemetry for precharge aborted??

            // CurrentBMSStatus.precharging = false;
            // CurrentBMSStatus.prechargeDone = false;
            prechargeState = PRECHARGE_FAULT;
        } else if (prechargeComplete(packVoltage)) {
            BMSInstance.nPrechargeControl = 1;
            // prechargeRelay = 0;

            precharging = false;
            prechargeDone = true;
            BMSInstance.currentState = BMSInstance.ACTIVE;
            // CurrentBMSStatus.precharging = false;
            // CurrentBMSStatus.prechargeDone = true;
            prechargeTimer.stop();
            prechargeState = PRECHARGE_IDLE;
        } else if (prechargeTimer.read() > PRECHARGE_TIMEOUT) {
            BMSInstance.nPrechargeControl = 1;
            // prechargeRelay = 0;
            precharging = false;
            prechargeDone = false;

            // also adding bms should fault in this case
            BMSInstance.currentState = BMSInstance.FAULT;
            // CurrentBMSStatus.precharging = false;
            // CurrentBMSStatus.prechargeDone = false;
            prechargeState = PRECHARGE_FAULT;
        }
        break;

    case PRECHARGE_FAULT:
        BMSInstance.nPrechargeControl = 1;

        // prechargeRelay = 0;
        precharging = false;
        prechargeDone = false;
        // CurrentBMSStatus.precharging = false;
        // CurrentBMSStatus.prechargeDone = false;
        if (!shutdownClosed()) {
            prechargeState = PRECHARGE_IDLE;
        }
        break;
    }
}
