#include "coulomb_counting.h"
#include <algorithm>
#include <stdexcept>

CoulombCounter::CoulombCounter(double soc_initial_pct,
                               double capacity_ah,
                               double eta_charge,
                               double eta_discharge,
                               double dt_s)
    : soc_(soc_initial_pct),
      Cbatt_(capacity_ah),
      eta_charge_(eta_charge),
      eta_discharge_(eta_discharge),
      dt_(dt_s),
      prev_current_(0.0),
      first_sample_(true)
{
    if (soc_initial_pct < 0.0 || soc_initial_pct > 100.0)
        throw std::invalid_argument("SOC must be between 0 and 100");

    if (capacity_ah <= 0.0)
        throw std::invalid_argument("capacity must be positive");
}

double CoulombCounter::update(double current_A)
{
    if (first_sample_) {
        prev_current_ = current_A;
        first_sample_ = false;
        return soc_;
    }

    double avg_current = (prev_current_ + current_A) / 2.0;
    double eta = (avg_current >= 0.0) ? eta_discharge_ : eta_charge_;

    double delta_soc =
        (eta / (3600.0 * Cbatt_)) * avg_current * dt_ * 100.0;

    soc_ -= delta_soc;
    soc_ = std::clamp(soc_, 0.0, 100.0);

    prev_current_ = current_A;
    return soc_;
}

void CoulombCounter::reset(double soc_initial_pct)
{
    soc_ = soc_initial_pct;
    prev_current_ = 0.0;
    first_sample_ = true;
}

double CoulombCounter::soc() const
{
    return soc_;
}