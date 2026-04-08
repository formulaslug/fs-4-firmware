#pragma once
#include <vector>

class CoulombCounter {
public:
    CoulombCounter(double soc_initial_pct,
                   double capacity_ah,
                   double eta_charge,
                   double eta_discharge,
                   double dt_s);

    // update SOC with one current sample
    double update(double current_A);

    void reset(double soc_initial_pct);

    double soc() const;

private:
    double soc_;
    double Cbatt_;
    double eta_charge_;
    double eta_discharge_;
    double dt_;
    double prev_current_;
    bool   first_sample_;
};