#pragma once

#include "BMS.h"
#include "mbed.h"

class prechargeLogic {
public:
 enum precharge_state { PRECHARGE_IDLE, PRECHARGE_ACTIVE, PRECHARGE_FAULT, PRECHAGE_COMPLETE };

 prechargeLogic(BMS* currentInstance);
 
float readDcBusVoltage();
float readPackVoltage();
bool shutdownClosed();
bool imdOk();
bool glvOk();
bool isBmsFaultActive();
bool preChargeAllowed();
bool prechargeComplete(float packVoltage);
void updatePrecharge(); 
void updatePackVoltage();

private:
    const float PRECHARGE_TIMEOUT = 3.0f;

    BMS* BMSInstance;

    Timer prechargeTimer;

    float dcBusVoltage;
    float packVoltage;   
    bool precharging = false;
    bool prechargeDone = false;

    precharge_state prechargeState = PRECHARGE_IDLE;


};
   