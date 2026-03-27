#ifndef ETC_CONTROLLER_H
#define ETC_CONTROLLER_H

#include "mbed.h"
#include <cstdint>

struct ETCState {
    float apps1Voltage;
    float apps2Voltage;
    float bppsVoltage;
    float travel1;
    float travel2;
    float pedalTravel;
    bool appsBoundsOk;
    bool mismatchOk;
    bool brakePressed;
    bool implausLatched;
    bool rtdEnabled;
    bool tsActive;
    bool torqueAllowed;
};

using AppsReadings = ETCState;

class ETCController {
public:
    ETCController();

    ETCState sample();

    ETCState readApps() { return sample(); }

    void setTSActive(bool active);

    bool impState() const;

    void clearImp();

    void TSActive(bool active);

    bool rtdState() const;

    void updateRTD();

    bool brakePressed();

    void startRTD();
    void stopRTD();
    bool torqueAllowed() const;

    bool isRTDEnabled() const { return rtdState(); }
    bool isTorqueAllowed() const { return torqueAllowed(); }

private:
    AnalogIn apps1_input;
    AnalogIn apps2_input;
    AnalogIn bpps_input;

    InterruptIn rtd_button;

    DigitalOut rtd_output;

    Timeout rtds_timeout;

    bool ts_active = false;
    bool rtd_enabled = false;
    volatile bool rtd_request = false;

    static constexpr float rtd_duration = 2.0f; //need to verify these values before testing.
    static constexpr float bPressedThresh = 0.4f;

    static constexpr float apps1MinV = 0.3125f;
    static constexpr float apps1MaxV = 2.8125f;

    static constexpr float apps2MinV = 0.1875f;
    static constexpr float apps2MaxV = 1.6875f;

    static constexpr float boundMargin = 0.05f;
    static constexpr float mismatchTol = 0.10f; 
    static constexpr float impTime = 0.100f; 

    Timer implausTimer;
    bool implausTimerRunning = false;
    bool implausLatched = false;

    static float clamp(float x);
    static bool inRange(float v, float lo, float hi, float margin);

    void onRTDButtonPressed();
    void stopRTDSound();
};

#endif