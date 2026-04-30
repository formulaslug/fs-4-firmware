#include "kalmanfilter.h"
#include <algorithm>    // std::clamp
#include <stdexcept>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
KalmanSOC::KalmanSOC(double soc_initial_pct,
                     double capacity_ah,
                     double dt_s,
                     double P0,
                     double Q,
                     double R,
                     double voltage_update_interval_s,
                     double eta_charge,
                     double eta_discharge)
    : soc_est_(soc_initial_pct),
      P_(P0),
      Q_(Q),
      R_(R),
      voltage_update_interval_s_(voltage_update_interval_s),
      voltage_timer_s_(0.0),
      dt_s_(dt_s),
      capacity_ah_(capacity_ah),
      cc_(soc_initial_pct, capacity_ah, eta_charge, eta_discharge, dt_s)
{
    if (soc_initial_pct < 0.0 || soc_initial_pct > 100.0)
        throw std::invalid_argument("KalmanSOC: initial SOC must be 0..100");
    if (capacity_ah <= 0.0)
        throw std::invalid_argument("KalmanSOC: capacity must be positive");
    if (dt_s <= 0.0)
        throw std::invalid_argument("KalmanSOC: dt must be positive");
}

// ---------------------------------------------------------------------------
// update  — call once every dt_s seconds
// ---------------------------------------------------------------------------
double KalmanSOC::update(double current_A, double voltage_V)
{
    // ------------------------------------------------------------------
    // STEP 1 & 2  Prediction  (coulomb counting)
    //
    //   CoulombCounter::update() does trapezoidal integration and returns
    //   the new SOC in percent [0..100].  This is our process model.
    // ------------------------------------------------------------------
    double soc_pred = cc_.update(current_A);

    // ------------------------------------------------------------------
    // STEP 2b  Grow estimation uncertainty each time-step
    // ------------------------------------------------------------------
    P_               += Q_;
    voltage_timer_s_ += dt_s_;

    // ------------------------------------------------------------------
    // STEP 3  Measurement update (voltage LUT) — every N seconds
    // ------------------------------------------------------------------
    if (voltage_timer_s_ >= voltage_update_interval_s_)
    {
        double soc_meas = voltageToSoc(voltage_V, current_A, capacity_ah_);

        if (soc_meas >= 0.0)   // -1 means the operating point was out of LUT bounds
        {
            // --------------------------------------------------------------
            // STEP 4  Kalman gain
            //   K close to 1 → trust the voltage measurement more
            //   K close to 0 → trust the coulomb-counting prediction more
            // --------------------------------------------------------------
            double K = P_ / (P_ + R_);

            // --------------------------------------------------------------
            // STEP 5  Correct prediction with voltage measurement
            // --------------------------------------------------------------
            soc_pred = soc_pred + K * (soc_meas - soc_pred);

            // --------------------------------------------------------------
            // STEP 6  Shrink uncertainty now that we have a measurement
            // --------------------------------------------------------------
            P_ = (1.0 - K) * P_;
        }
        // else: LUT lookup failed (voltage out of range) — keep prediction only

        voltage_timer_s_ = 0.0;
    }

    // ------------------------------------------------------------------
    // STEP 7  Clamp to valid range and store
    // ------------------------------------------------------------------
    soc_est_ = std::clamp(soc_pred, 0.0, 100.0);
    return soc_est_;
}

// ---------------------------------------------------------------------------
// voltageToSoc  — converts terminal voltage + current → SOC % via linInterp2D
// ---------------------------------------------------------------------------
double KalmanSOC::voltageToSoc(double voltage_V, double current_A, double capacity_ah)
{
    // Your LUT operates in milli-units
    DischargeDataPoint point;
    point.milliVolts    = static_cast<int>(voltage_V * 1000.0);
    point.milliAmps     = static_cast<int>(current_A * 1000.0);
    point.milliAmpHours = 0;   // output — filled by linInterp2D

    if (linInterp2D(&point) == -1) {
        return -1.0;   // operating point outside LUT — caller ignores measurement
    }

    // milliAmpHours from the LUT = *remaining* capacity at this operating point.
    // SOC % = (remaining_mAh / nominal_mAh) * 100
    double nominal_mAh = capacity_ah * 1000.0;
    double soc_pct     = (static_cast<double>(point.milliAmpHours) / nominal_mAh) * 100.0;

    return std::clamp(soc_pct, 0.0, 100.0);
}

// ---------------------------------------------------------------------------
// reset  — use after a known full charge or manual SOC correction
// ---------------------------------------------------------------------------
void KalmanSOC::reset(double soc_initial_pct)
{
    soc_est_         = soc_initial_pct;
    P_               = 0.02;
    voltage_timer_s_ = 0.0;
    cc_.reset(soc_initial_pct);
}