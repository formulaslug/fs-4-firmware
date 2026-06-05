#pragma once

/**
 * kalman_soc.h
 *
 * Kalman filter SOC estimator that fuses:
 *   - Prediction  : CoulombCounter  (coulomb counting, runs every dt_s)
 *   - Measurement : voltage_lookup  (linInterp2D LUT, sampled every voltage_update_interval_s)
 *
 * Units contract (matches your existing code):
 *   current   -> Amperes    (positive = discharging, matching CoulombCounter convention)
 *   voltage   -> Volts
 *   SOC       -> percent [0, 100]
 *   capacity  -> Amp-hours
 */

#include "coulomb_counting.h"   // CoulombCounter
#include "voltage_lookup.h"     // linInterp2D, DischargeDataPoint
#include "batteryLUT.h"         // LUT constants

class KalmanSOC {
public:
    /**
     * @param soc_initial_pct           Starting SOC estimate            [0..100]
     * @param capacity_ah               Nominal pack capacity             [Ah]
     * @param dt_s                      Fixed time-step                   [s]
     * @param P0                        Initial estimation uncertainty    (try 0.02)
     * @param Q                         Process noise  — lower = trust CC more     (try 0.001)
     * @param R                         Measurement noise — lower = trust LUT more (try 0.01)
     * @param voltage_update_interval_s How often to run the voltage correction    (default 5 s)
     * @param eta_charge                Coulombic efficiency while charging         (default 0.99)
     * @param eta_discharge             Coulombic efficiency while discharging      (default 1.0)
     */
    KalmanSOC(double soc_initial_pct,
              double capacity_ah,
              double dt_s,
              double P0                        = 0.02,
              double Q                         = 0.001,
              double R                         = 0.01,
              double voltage_update_interval_s  = 5.0,
              double eta_charge                = 0.99,
              double eta_discharge             = 1.0);

    /**
     * Call once per time-step with the latest sensor readings.
     *
     * @param current_A  Pack current in Amperes (+ discharge / – charge)
     * @param voltage_V  Pack terminal voltage in Volts
     * @return           Fused SOC estimate [0..100]
     */
    double update(double current_A, double voltage_V);

    /** Hard-reset the filter (e.g. after a known full charge). */
    void reset(double soc_initial_pct);

    // ---- accessors ----
    double soc()          const { return soc_est_; }
    double uncertainty()  const { return P_; }
    double voltageTimer() const { return voltage_timer_s_; }

private:
    // Kalman state
    double soc_est_;                     // fused SOC estimate [0..100]
    double P_;                           // estimation error covariance

    // Noise parameters
    double Q_;                           // process noise covariance
    double R_;                           // measurement noise covariance

    // Timing
    double dt_s_;                        // fixed time-step [s]
    double voltage_update_interval_s_;
    double voltage_timer_s_;

    // Config
    double capacity_ah_;

    // Sub-modules
    CoulombCounter cc_;                  // handles coulomb-counting prediction

    /**
     * Convert voltage + current → SOC [0..100] via the 2-D LUT (linInterp2D).
     * Returns -1.0 if the operating point is outside the LUT bounds.
     */
    double voltageToSoc(double voltage_V, double current_A, double capacity_ah);
};