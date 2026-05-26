//
// Created by Trey Lutton, 4/14/2026
//
#ifndef TRACTION_CONTROL_H
#define TRACTION_CONTROL_H

#include "mbed.h"

class TractionController {
  public:
    TractionController();

    // main PID update loop - returns a reduction factor to be multiplied with the requested torque
    float update(float ws_fl, float ws_fr, float ws_rl, float ws_rr);

    // getters (for logging)
    float get_slip() const;
    float get_integral() const;
    float get_raw_derivative() const;
    float get_smoothed_derivative() const;
    float get_loop_time() const;
    float get_output() const;

  private:
    // TUNING VARIABLES
    static constexpr float KP = 1.0f;                                // strength of proportional component
    static constexpr float KI = 0.0f;                                // strength of integral component
    static constexpr float KD = 0.0f;                                // strength of derivative component
    static constexpr float MAX_INTEGRATOR_REDUCTION_FACTOR = 0.5f;   // maximum strength of integral component ALONE
    static constexpr float TARGET_WHEEL_SLIP = 0.09f;                // wheel slip the controller will try to maintain 
    static constexpr float ACTIVATION_RPM = 100.0f;                  // min speed in wheel rpm
    static constexpr float FILTER_CUTOFF_FREQ = 1.0f;                // TODO: choose cutoff frequency
    static constexpr float MIN_OUTPUT = 0.2f;                        // min reduction factor
    static constexpr float MAX_OUTPUT = 1.0f;                        // max reduction factor (DO NOT INCREASE)
  
    float slip;
    float integral;  
    float loop_time;
    float prev_slip;
    float last_output;
    float raw_derivative;
    float smoothed_derivative;
    bool prev_slip_stale;
    bool filter_init;
    float filter_time_constant;
    float max_integral;

    Timer loop_timer; 

    void reset();
};

#endif
