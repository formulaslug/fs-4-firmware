#ifndef PRECHARGELOGIC_H
#define PRECHARGELOGIC_H

#include "mbed.h"
#include "BmsConfig.h"

enum precharge_state {
    PRECHARGE_IDLE,
    PRECHARGE_ACTIVE,
    PRECHARGE_FAULT
};

extern bool precharging;
extern bool prechargeDone;
extern float dcBusVoltage;
extern float packVoltage;
extern precharge_state prechargeState;

void updatePrecharge();
bool preChargeAllowed();
bool prechargeComplete();
bool glvOk();
bool isBmsFaultActive();

#endif