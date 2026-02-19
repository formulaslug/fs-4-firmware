#include "etc_controller.h"
#include <cmath>
#include <cstdint>

ETCController::ETCController()
: apps1_input(PC_1),
  apps2_input(PC_2)
{}

float ETCController::clamp(float x){
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

bool ETCController::inRange(float v, float lo, float hi, float margin) {
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
    AppsReadings data;

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

    if(implausLatched){
        return data;
    }

    if(impCondition){
        if(!implausTimerRunning){
            implausTimer.reset();
            implausTimer.start();
            implausTimerRunning = true;
        }

        const float t = std::chrono::duration<float>(implausTimer.elapsed_time()).count();

        if (t >= impTime){
            implausLatched = true;
            implausTimer.stop();
            implausTimerRunning = false;
        }
    }else{
        if(implausTimerRunning){
            implausTimer.stop();
            implausTimer.reset();
            implausTimerRunning = false;
        }
    }
    return data;
}
