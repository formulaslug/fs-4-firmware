#include "coulomb_counting.h"
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char const* argv[]) {
    // arg1: previous SOC percent
    // arg2: battery capacity in Ah
    // arg3: charge efficiency
    // arg4: discharge efficiency
    // arg5: dt in seconds
    // arg6: previous current in A
    // arg7: current in A
    // output: updated SOC percent

    if (argc != 8) {
        throw std::invalid_argument("Expected 7 arguments, got: " + std::to_string(argc - 1));
    }

    double prev_soc_pct;
    double capacity_ah;
    double eta_charge;
    double eta_discharge;
    double dt_s;
    double prev_current_A;
    double current_A;

    try {
        prev_soc_pct   = std::stod(argv[1]);
        capacity_ah    = std::stod(argv[2]);
        eta_charge     = std::stod(argv[3]);
        eta_discharge  = std::stod(argv[4]);
        dt_s           = std::stod(argv[5]);
        prev_current_A = std::stod(argv[6]);
        current_A      = std::stod(argv[7]);
    } catch (...) {
        throw std::invalid_argument("Expected seven numeric arguments");
    }

    CoulombCounter counter(
        prev_soc_pct,
        capacity_ah,
        eta_charge,
        eta_discharge,
        dt_s
    );

    counter.update(prev_current_A);
    double updated_soc = counter.update(current_A);

    std::cout << updated_soc << std::endl;
    return 0;
}