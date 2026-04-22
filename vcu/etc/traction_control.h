//
// Created by Trey Lutton, 4/14/2026
//
#ifndef TRACTION_CONTROL_H
#define TRACTION_CONTROL_H

#include "mbed.h"

class TractionController {
  public:
    TractionController();
    float get_reduction_factor(int16_t ws_fl, int16_t ws_fr, int16_t ws_rl, int16_t ws_rr);
    void reset();
    float get_last_error();
    float get_last_loop_time();

  private:
    // TUNING VARIABLES
    static constexpr float KP = 1.0f;                 // strength of proportional component
    static constexpr float KD = 0.0f;                 // strength of derivative component
    static constexpr float TARGET_WHEEL_SLIP = 0.09f; // 
    static constexpr float ACTIVATION_RPM = 100.0f;   // min speed in wheel rpm
    static constexpr float MIN_OUTPUT = 0.2f;         // min reduction factor
    static constexpr float MAX_OUTPUT = 1.0f;         // max reduction factor (DO NOT INCREASE)
    
    float last_loop_time;
    float prev_error;
    bool prev_error_stale;
    Timer loop_timer; 
};

#endif
