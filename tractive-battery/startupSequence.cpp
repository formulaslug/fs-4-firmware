#include "startupSequence.h"
#include "BmsConfig.h"
#include "mbed.h"

DigitalOut prechargeRelay(PRECHARGECTRL);
AnalogIn glvVoltage(GLVMEASURE);
DigitalIn imdFaultPin(IMDFAULT);
DigitalIn shutdownPin(SHUTDOWNMEASURE);

bool precharging = false;
bool prechargeDone = false;

float dcBusVoltage = 0.0;   
float packVoltage  = 400.0;

precharge_state prechargeState = PRECHARGE_IDLE;
Timer prechargeTimer;
const float PRECHARGE_TIMEOUT = 3.0f; //placeholder timeout value in seconds

EventQueue queue(32 * EVENTS_EVENT_SIZE);

float readDcBusVoltage() {
    return dcBusVoltage; 
}

bool glvOk() {
    glvVoltage = (uint16_t)(glv_voltage_pin * 3.3 * 21.9/3.9 * 1000);
    CurrentBMSStatus.glv_voltage = (uint16_t)(glv * 1000);
    return (glv > 11.0f && glv < 15.0f); //place holder 
}

bool isBmsFaultActive() {
    return CurrentBMSStatus.bmsFault;
}

bool shutdownClosed() {
    return shutdownPin.read();
}

bool imdOk() {
    return imdFaultPin.read();
}

bool preChargeAllowed() {
    return shutdownClosed() && imdOk() && !isBmsFaultActive() && glvOk();
}

bool prechargeComplete(float packVoltage) {
    return dcBusVoltage >= 0.9f * packVoltage;
}

void updatePrecharge() {
    switch (prechargeState) {
        case PRECHARGE_IDLE:
            precharging = false;
            prechargeDone = false;
            CurrentBMSStatus.precharging = false;
            CurrentBMSStatus.prechargeDone = false;
            if (preChargeAllowed() && packVoltage >= MIN_PACK_MV) {
                prechargeRelay = 1;
                precharging = true;
                CurrentBMSStatus.precharging = true;
                CurrentBMSStatus.prechargeDone = false;
                prechargeTimer.reset();
                prechargeTimer.start();
                prechargeState = PRECHARGE_ACTIVE;
            }
            break;

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
                prechargeState = PRECHARGE_IDLE;
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

int main() {
    prechargeRelay = 0;
    prechargeTimer.stop();

    queue.call_every(10ms, updatePrecharge);
    queue.dispatch_forever();
}