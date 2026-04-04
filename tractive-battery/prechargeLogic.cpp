#include "prechargeLogic.h"
#include "BmsConfig.h"
#include "mbed.h"


/*
I think some major rewrites are needed here, i dont think the logic fully holds up
I think it would be simpler and better to have 3 states
PRECHARGE_ACTIVE
PRECHARGE_IDLE
PRECHARGE_FAULT


i dont think we really need a state to tell us when the precharge is complete, ideally thats always idle when the car is running ... 

no global variables should be defined here nor should an event queue be defined here

potential logic issues: 
 if we are in precharge idle state we start precharging if we have no faults (prechargeAllowed returns true, and the pack voltage is okay),
 the issue is that we just close the precharge relay no matter what if were allowed to, and we dont check dcBusVoltage later on it does check,
 when its in PRECHARGE_ACTIVE but i think we should always know the dcBusVoltage before we close the precharge relay....


 the other issue is that once we are in the PRECHARGE_COMPLETE state the only way we can ever precharge again is if we have a fault somewhere, which only happens if:
 prechargeAllowed returns false - note that precharge allowed does not check the dcBusVoltage
 
 i think the assumption might have been we precharge once on startup, but this isnt true. 




the cars dc voltage can drop under high load during driving, so we need to be able to precharge the car while its actually running. we should assume we need to do this somewhat often 


i think its worth it to do a full rewrite, going to double check what is needed then maybe make fsm model so its clear what needs to be implemented 





*/



bool precharging = false;
bool prechargeDone = false;

float dcBusVoltage = 0.0;   
float packVoltage  = 400.0; // shouldnt be set just here....

precharge_state prechargeState = PRECHARGE_IDLE;
Timer prechargeTimer;
const float PRECHARGE_TIMEOUT = 3.0f; //placeholder timeout value in seconds


float readDcBusVoltage() {
    return dcBusVoltage;  //this is not correct has to be read from can, but we can use fs3 implementation for now  
}

bool glvOk(BMS& BMSInstance) {
    glvVoltage = (uint16_t)(BMSInstance.glvVoltage.read() * 3.3 * 21.9/3.9 * 1000);
    //CurrentBMSStatus.glv_voltage = (uint16_t)(glv * 1000);
    BMSInstance.bms_stat_message.glv_voltage = glvVoltage;
    return (glv > 11.0f && glv < 15.0f); //place holder for 11 to 15 volts? we should find out the limit  
}

bool isBmsFaultActive(BMS &BMSInstance) {
    return BMSInstance.currentState;
}

bool shutdownClosed(BMS &BMSInstance) {
    return BMSInstance.nBMS_Fault_3V3.read();
    //we also need to add safety checks here to make sure we dont precharge when the shutdown circuit is open anywhere else 
}

bool imdOk(BMS &BMSInstance) {
    // return imdFaultPin.read();
    // i think this should probably be in BMSFaultDetection, but for
    return BMSInstance.IMD_Fault_3V3.read();
}

bool preChargeAllowed(BMS &BMSInstance) {
    return shutdownClosed(BMSInstance) && imdOk(BMSInstance) && !isBmsFaultActive(BMSInstance) && glvOk(BMSInstance);
}

bool prechargeComplete(float packVoltage) {
    return dcBusVoltage >= 0.9f * packVoltage; // i think this is good i wanna double check tho
}

void updatePrecharge(BMS &BMSInstance) {
    switch (prechargeState) {
        case PRECHARGE_IDLE:
            precharging = false;
            prechargeDone = false;
            if(BMSInstance.currentState==BMSInstance.ACTIVE){ // added by Ethan going to be removed later 
            // CurrentBMSStatus.prechargeDone = false;
                /*
                    I dont think the logic is correct here, does it just automatically precharge if the battery packs are okay and its not already precharging?
                    theres no checks to see if we have a good dcBusVoltage in precharge allowed.....
                */
                if (preChargeAllowed(BMSInstance) && packVoltage >= MIN_PACK_MV) {
                    prechargeRelay = 1;
                    precharging = true;
                    CurrentBMSStatus.precharging = true;
                    CurrentBMSStatus.prechargeDone = false;
                    prechargeTimer.reset();
                    prechargeTimer.start();
                    prechargeState = PRECHARGE_ACTIVE;
                }
                break;
            }

        case PRECHARGE_ACTIVE:
            if (!preChargeAllowed()) {
                prechargeRelay = 0;
                precharging = false;
                prechargeDone = false;
                CurrentBMSStatus.precharging = false;
                CurrentBMSStatus.prechargeDone = false;
                prechargeState = PRECHARGE_FAULT;
            } 
            else if (prechargeComplete(packVoltage)) {
                prechargeRelay = 0;
                precharging = false;
                prechargeDone = true;
                CurrentBMSStatus.precharging = false;
                CurrentBMSStatus.prechargeDone = true;
                prechargeTimer.stop();
                prechargeState = PRECHARGE_COMPLETE;
            } 
            else if (prechargeTimer.read() > PRECHARGE_TIMEOUT) {
                prechargeRelay = 0;
                precharging = false;
                prechargeDone = false;
                CurrentBMSStatus.precharging = false;
                CurrentBMSStatus.prechargeDone = false;
                prechargeState = PRECHARGE_FAULT;
            }
            break;

        case PRECHARGE_COMPLETE:
            precharging = false;
            prechargeDone = true;
            CurrentBMSStatus.precharging = false;
            CurrentBMSStatus.prechargeDone = true;
            if (!preChargeAllowed()) {
                prechargeDone = false;
                CurrentBMSStatus.prechargeDone = false;
                prechargeState = PRECHARGE_IDLE; // why does it only go to idle here, we can only precharge again if we fault somehow? 
            }
            break;

        case PRECHARGE_FAULT:
            prechargeRelay = 0;
            precharging = false;
            prechargeDone = false;
            CurrentBMSStatus.precharging = false;
            CurrentBMSStatus.prechargeDone = false;
            if (!shutdownClosed()) {
                prechargeState = PRECHARGE_IDLE;
            }
            break;
    }
}

