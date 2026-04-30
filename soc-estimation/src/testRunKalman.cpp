/**
 * testRunKalmanFilter.cpp
 *
 * Stateless per-call wrapper around KalmanSOC — mirrors the pattern of
 * testRunCoulombCountingRunner so testRunData.py can carry state in Python.
 *
 * Args (positional):
 *   1  prev_soc_pct   Previous SOC estimate carried in from Python  [0..100]
 *   2  capacity_ah    Nominal pack capacity                          [Ah]
 *   3  eta_charge     Coulombic efficiency while charging
 *   4  eta_discharge  Coulombic efficiency while discharging
 *   5  dt_s           Time-step                                      [s]
 *   6  prev_current_A Current at t-1                                 [A]
 *   7  current_A      Current at t                                   [A]
 *   8  voltage_V      Terminal voltage at t                          [V]
 *   9  P              Covariance carried in from Python              (default 0.02)
 *   10 Q              Process noise                                  (default 0.001)
 *   11 R              Measurement noise                              (default 0.01)
 *   12 interval_s     Voltage correction interval                    (default 5.0)
 *
 * Stdout (one line):
 *   <soc_pct>,<uncertainty_P>
 *
 * Python carries prev_soc_pct and P between calls so the filter is
 * mathematically continuous even though the exe is stateless.
 *
 * KalmanSOC is initialised with prev_soc_pct and P, then its internal
 * CoulombCounter is primed with prev_current_A (trips the first-sample
 * guard), and finally update(current_A, voltage_V) runs the full step.
 *
 * Build:
 *   g++ -std=c++17 -O2 -o testRunKalmanFilter \
 *       testRunKalmanFilter.cpp kalman_soc.cpp coulomb_counting.cpp voltage_lookup.cpp
 */

#include <iostream>
#include <algorithm>
#include <cstdlib>

#include "kalmanfilter.h"   // KalmanSOC — pulls in coulomb_counting.h and voltage_lookup.h

int main(int argc, char* argv[])
{
    if (argc < 9) {
        std::cerr
            << "Usage: testRunKalmanFilter"
               " <prev_soc_pct> <capacity_ah> <eta_charge> <eta_discharge>"
               " <dt_s> <prev_current_A> <current_A> <voltage_V>"
               " [P=0.02] [Q=0.001] [R=0.01] [interval_s=5.0]\n";
        return 1;
    }

    double prev_soc_pct  = std::stod(argv[1]);
    double capacity_ah   = std::stod(argv[2]);
    double eta_charge    = std::stod(argv[3]);
    double eta_discharge = std::stod(argv[4]);
    double dt_s          = std::stod(argv[5]);
    double prev_current  = std::stod(argv[6]);
    double current_A     = std::stod(argv[7]);
    double voltage_V     = std::stod(argv[8]);

    double P          = (argc > 9)  ? std::stod(argv[9])  : 0.02;
    double Q          = (argc > 10) ? std::stod(argv[10]) : 0.001;
    double R          = (argc > 11) ? std::stod(argv[11]) : 0.01;
    double interval_s = (argc > 12) ? std::stod(argv[12]) : 5.0;

    // Construct KalmanSOC with the carried-in state.
    // P0 is set to the carried-in P so covariance is continuous across calls.
    KalmanSOC kf(prev_soc_pct, capacity_ah, dt_s,
                 /*P0=*/P, Q, R, interval_s,
                 eta_charge, eta_discharge);

    // Prime the internal CoulombCounter with prev_current_A.
    // This trips its first-sample guard so the next update() call performs
    // the real trapezoidal integration between prev_current and current_A.
    kf.update(prev_current, voltage_V);

    // Full Kalman step: coulomb counting prediction + voltage correction.
    double soc = kf.update(current_A, voltage_V);

    // soc_pct and P are read back by Python to seed the next call.
    std::cout << soc << "," << kf.uncertainty() << "\n";
    return 0;
}