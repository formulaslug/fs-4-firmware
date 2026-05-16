#include "prechargeLogic.h"
// #include "BMS.h"
#include "mbed.h"
// #include "BMSFaultDetection.h"


prechargeLogic::prechargeLogic(BMS* currentInstance){
    BMSInstance = currentInstance;
    dcBusVoltage = readDcBusVoltage();
    updatePackVoltage();
}  


void prechargeLogic::updatePackVoltage(){
    packVoltage = BMSInstance->currentBatteryVoltage;
}

float prechargeLogic::readDcBusVoltage() {
    //fs3 implentation 
    BMSInstance->CAN_POWERTRAIN.read(BMSInstance->msg);
    uint32_t id = BMSInstance->msg.id;
    unsigned char* data = BMSInstance->msg.data;
    if (!BMSInstance->Charge_State_Filtered.read()) {
        switch (id) { // copied over from fs3
        case 0x682:   // temperature message from MC
            dcBusVoltage = (data[2] | (data[3] << 8));
            break;
        default:
            break;
        }
    }
    return dcBusVoltage; 
}

bool prechargeLogic::glvOk() {
    float glvVoltage = (uint16_t)(BMSInstance->GLV_Voltage.read() * 3.3 * 21.9 / 3.9 * 1000);
    // CurrentBMSStatus.glv_voltage = (uint16_t)(glv * 1000);
    //  BMSInstance->bms_stat_message.glv_voltage = glvVoltage;
    return (
        glvVoltage > 11.0f && glvVoltage < 15.0f
    ); // place holder for 11 to 15 volts? we should find out the limit
}

bool prechargeLogic::isBmsFaultActive() { return BMSInstance->currentState; }

bool prechargeLogic::shutdownClosed() {
        return BMSInstance->Shutdown_Measure.read(); 
}

bool prechargeLogic::imdOk() {
    return BMSInstance->IMD_Fault_3V3.read();
}

bool prechargeLogic::preChargeAllowed() {
    return shutdownClosed()
           && !isBmsFaultActive()
           && glvOk()
           && dcBusVoltage < 0.9f * packVoltage;
}
bool prechargeLogic::prechargeComplete() {
    return dcBusVoltage >= 0.9f * packVoltage; // i think this is good i wanna double check tho
}

void prechargeLogic::updatePrecharge() {
    readDcBusVoltage(); // calling it here to make sure we have the most recent value for dcBusVoltage
    updatePackVoltage(); // want the most accurate pack voltage as well
    switch (prechargeState) {
    case PRECHARGE_IDLE:
        precharging = false;
            if (preChargeAllowed() && packVoltage >= MIN_PACK_MV && !prechargeComplete()) {
                BMSInstance->nPrechargeControl = 0; // outputs low during precharge!
                // prechargeRelay = 1;
                precharging = true;
                // CurrentBMSStatus.precharging = true;
                // CurrentBMSStatus.prechargeDone = false;
                BMSInstance->currentState = BMSInstance->PRECHARGING;
                prechargeTimer.reset();
                prechargeTimer.start();
                prechargeState = PRECHARGE_ACTIVE;
            }
        break;


    case PRECHARGE_ACTIVE:
        if (!preChargeAllowed()) {
            BMSInstance->nPrechargeControl = 1; // outputs hight to stop precharge
            // prechargeRelay = 0;
            precharging = false;
            prechargeDone = false;
            // might need to add some telemetry for precharge aborted??
            prechargeTimer.stop();
            prechargeTimer.reset();
            // BMSInstance->currentState = BMSInstance->FAULT;
            prechargeState = PRECHARGE_FAULT;


            // CurrentBMSStatus.precharging = false;
            // CurrentBMSStatus.prechargeDone = false;
        } else if (prechargeComplete()) {
            BMSInstance->nPrechargeControl = 1;
            // prechargeRelay = 0;

            precharging = false;
            prechargeDone = true;
            BMSInstance->currentState = BMSInstance->ACTIVE; // i have a feeling that this needs to be detected and updated by the bms instead...
            // CurrentBMSStatus.precharging = false;
            // CurrentBMSStatus.prechargeDone = true;
            prechargeTimer.stop();
            prechargeState = PRECHARGE_COMPLETE;

        } else if (prechargeTimer.read() > PRECHARGE_TIMEOUT) {
            BMSInstance->nPrechargeControl = 1;
            // prechargeRelay = 0;
            precharging = false;
            prechargeDone = false;
            prechargeTimer.stop();
            prechargeTimer.reset();

            // also adding bms should fault in this case
            BMSInstance->currentState = BMSInstance->FAULT;
            // CurrentBMSStatus.precharging = false;
            // CurrentBMSStatus.prechargeDone = false;
            prechargeState = PRECHARGE_FAULT;
        }
        break;
    case PRECHARGE_COMPLETE:
        // BMSInstance->nPrechargeControl = 1;
        if(!(preChargeAllowed())){
            prechargeState = PRECHARGE_IDLE; 
            //we set this to idle here .... this is because we ideally want to recover from a fault 
            //so precharge will get called on startup and when there is a fault recover
        }
        break;


    case PRECHARGE_FAULT:
        BMSInstance->nPrechargeControl = 1;

        // prechargeRelay = 0;
        precharging = false;
        prechargeDone = false;
        // CurrentBMSStatus.precharging = false;
        // CurrentBMSStatus.prechargeDone = false;
        if (!shutdownClosed()) {
            prechargeState = PRECHARGE_IDLE;
        }
        break;

    default: break;
    }
}
