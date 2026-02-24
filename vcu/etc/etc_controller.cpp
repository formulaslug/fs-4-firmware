#include "etc_controller.h"
#include <cmath>

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

AppsReadings ETCController::readApps(){
    AppsReadings data{};

    data.voltage1 = apps1_input.read_voltage();
    data.voltage2 = apps2_input.read_voltage();

    const bool ok1 = inRange(data.voltage1, apps1MinV, apps1MaxV, boundMargin);
    const bool ok2 = inRange(data.voltage2, apps2MinV, apps2MaxV, boundMargin);
    data.bounds_ok = ok1 && ok2;

    const float range1 = apps1MaxV - apps1MinV;
    const float range2 = apps2MaxV - apps2MinV;

    data.travel1 = clamp((data.voltage1 - apps1MinV) / range1);
    data.travel2 = clamp((data.voltage2 - apps2MinV) / range2);

    const float diff = fabsf(data.travel1 - data.travel2);
    data.mismatch_ok = (diff <= mismatchTol);

    data.pedal = 0.5f * (data.travel1 + data.travel2);

    const bool impCondition = (!data.bounds_ok) || (!data.mismatch_ok);

    if (implausLatched){
        return data;
    }

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
            implausTimerRunning = false;
            disableRTD();
        }
    }
    else{
        if (implausTimerRunning){
            implausTimer.stop();
            implausTimer.reset();
            implausTimerRunning = false;
        }
    }

    return data;
}

void ETCController::TSActive(bool active){
    ts_active = active;

    if (!ts_active){
        disableRTD();
        rtd_request = false;
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
        disableRTD();
        rtd_request = false;
        return;
    }

    if (!rtd_enabled){
        if (ts_active && brakePressed() && rtd_request){
            rtd_enabled = true;
            rtd_request = false;
            startRTD();
        }
    }
}

void ETCController::startRTD(){
    float dur = rtd_duration;
    if (dur < 1.0f) dur = 1.0f;
    if (dur > 3.0f) dur = 3.0f;

    rtd_output = 1;

    rtds_timeout.detach();
    rtds_timeout.attach(callback(this, &ETCController::stopRTD), dur);
}

void ETCController::stopRTD(){
    rtd_output = 0;
}

void ETCController::disableRTD(){
    rtd_enabled = false;
    rtd_output = 0;
    rtds_timeout.detach();
}