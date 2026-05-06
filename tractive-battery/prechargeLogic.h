#pragma once

#include "BMS.h"
#include "mbed.h"

class prechargeLogic {
public:
 enum precharge_state { PRECHARGE_IDLE, PRECHARGE_ACTIVE, PRECHARGE_FAULT };

 prechargeLogic(BMS& BMSinstance);
 
float readDcBusVoltage();
float readPackVoltage();
bool shutdownClosed();
bool imdOk();
bool glvOk();
bool isBmsFaultActive();
bool preChargeAllowed();
bool prechargeComplete(float packVoltage);
void updatePrecharge(); 

private:
    BMS& BMSinstance;

    Timer prechargeTimer;

    float dcBusVoltage = 0.0f;
    float packVoltage = 400.0f;
    bool precharging = false;
    bool prechargeDone = false;

    precharge_state prechargeState = PRECHARGE_IDLE;

    const float PRECHARGE_TIMEOUT = 3.0f;

};
   