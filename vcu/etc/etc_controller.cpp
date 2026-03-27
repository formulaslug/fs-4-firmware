#include "etc_controller.h"
#include <cmath>
#include <chrono>

ETCController::ETCController()
: apps1_input(PC_1),
  apps2_input(PC_2),
  bpps_input(PA_0),
  rtd_button(PB_0),
  rtd_output(PA_5)
{
    rtd_output = 0;
    rtd_button.fall(callback(this, &ETCController::onRTDButtonPressed));
}

float ETCController::clamp(float x){
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

bool ETCController::inRange(float v, float lo, float hi, float margin){
    return (v >= (lo - margin)) && (v <= (hi + margin));
}

bool ETCController::impState() const{
    return implausLatched;
}

void ETCController::clearImp(){
    implausLatched = false;
    implausTimer.stop();
    implausTimer.reset();
    implausTimerRunning = false;
}

ETCState ETCController::sample(){
    ETCState state{};

    state.apps1Voltage = apps1_input.read_voltage();
    state.apps2Voltage = apps2_input.read_voltage();
    state.bppsVoltage = bpps_input.read_voltage();

    const bool ok1 = inRange(state.apps1Voltage, apps1MinV, apps1MaxV, boundMargin);
    const bool ok2 = inRange(state.apps2Voltage, apps2MinV, apps2MaxV, boundMargin);
    state.appsBoundsOk = ok1 && ok2;

    const float range1 = apps1MaxV - apps1MinV;
    const float range2 = apps2MaxV - apps2MinV;

    state.travel1 = clamp((state.apps1Voltage - apps1MinV) / range1);
    state.travel2 = clamp((state.apps2Voltage - apps2MinV) / range2);

    state.mismatchOk = (fabsf(state.travel1 - state.travel2) <= mismatchTol);
    state.pedalTravel = 0.5f * (state.travel1 + state.travel2);
    state.brakePressed = (state.bppsVoltage >= bPressedThresh);
    state.tsActive = ts_active;
    state.implausLatched = implausLatched;
    state.rtdEnabled = rtd_enabled;
    state.torqueAllowed = torqueAllowed();

    if (!implausLatched){
        const bool impCondition = (!state.appsBoundsOk) || (!state.mismatchOk);

        if (impCondition){
            if (!implausTimerRunning){
                implausTimer.reset();
                implausTimer.start();
                implausTimerRunning = true;
            }

            const float t =
                std::chrono::duration<float>(implausTimer.elapsed_time()).count();

            if (t >= impTime){
                implausLatched = true;
                implausTimer.stop();
                implausTimer.reset();
                implausTimerRunning = false;
                stopRTD();
            }
        }
        else{
            if (implausTimerRunning){
                implausTimer.stop();
                implausTimer.reset();
                implausTimerRunning = false;
            }
        }
    }

    state.implausLatched = implausLatched;
    state.rtdEnabled = rtd_enabled;
    state.torqueAllowed = torqueAllowed();
    return state;
}

void ETCController::setTSActive(bool active){
    TSActive(active);
}

void ETCController::TSActive(bool active){
    ts_active = active;
    if (!ts_active){
        stopRTD();
    }
}

bool ETCController::rtdState() const{
    return rtd_enabled;
}

bool ETCController::brakePressed(){
    return (bpps_input.read() >= bPressedThresh);
}

void ETCController::onRTDButtonPressed(){
    rtd_request = true;
}

void ETCController::updateRTD(){
    if (!ts_active || implausLatched){
        stopRTD();
        return;
    }

    if (rtd_request && !brakePressed()){
        rtd_request = false;
    }

    if (!rtd_enabled){
        if (brakePressed() && rtd_request){
            rtd_request = false;
            startRTD();
        }
    }
}

void ETCController::startRTD(){
    rtd_enabled = true;

    float dur = rtd_duration;
    if (dur < 1.0f) dur = 1.0f;
    if (dur > 3.0f) dur = 3.0f;

    rtd_output = 1;

    rtds_timeout.detach();
    rtds_timeout.attach(callback(this, &ETCController::stopRTDSound), std::chrono::milliseconds(static_cast<int>(dur * 1000.0f)));
}

void ETCController::stopRTDSound(){
    rtd_output = 0;
}

void ETCController::stopRTD(){
    rtd_enabled = false;
    rtd_request = false;
    rtd_output = 0;
    rtds_timeout.detach();
}

bool ETCController::torqueAllowed() const{
    return (ts_active && rtd_enabled && !implausLatched);
}