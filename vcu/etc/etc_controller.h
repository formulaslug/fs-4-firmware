#ifndef ETC_CONTROLLER_H
#define ETC_CONTROLLER_H

#include "mbed.h"   
#include <cstdint>

struct AppsReadings{
    float voltage1;
    float voltage2;
    float travel1;
    float travel2;
    float pedal;
    bool bounds_ok;
    bool mismatch_ok;
};

class ETCController {
public:
    ETCController();

    AppsReadings readApps();

    bool impState() const;

    void clearImp();

private:
    AnalogIn apps1_input;
    AnalogIn apps2_input;

    static constexpr float apps1MinV  = 0.3125f;
    static constexpr float apps1MaxV = 2.8125f;
        
    static constexpr float apps2MinV  = 0.1875f;
    static constexpr float apps2MaxV = 1.6875f;

    static constexpr float boundMargin = 0.05f;

    static constexpr float mismatchTol = 0.10f;

    static constexpr float impTime = 0.100f;

    Timer implausTimer;
    bool implausTimerRunning = false;
    bool implausLatched = false;

    static float clamp(float x);

    static bool inRange(float v, float lo, float hi, float margin);
};




#endif //ETC_CONTROLLER_H