#include "voltage_lookup.h"
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char const* argv[]) {
    // arg1: milliVolts
    // arg2: milliAmps
    // outputs: linear interpolated value
    if (argc != 3) {
        throw std::invalid_argument("Expected 2 arguments, got: " + std::to_string(argc - 1));
    }

    // parse input as integers
    int milliVolts;
    int milliAmps;
    try {
        milliVolts = std::stoi(argv[1]);
        milliAmps = std::stoi(argv[2]);
    } catch (std::invalid_argument error) {
        throw std::invalid_argument("Expected two integers");
    }

    // feed to 2d interpolation function
    DischargeDataPoint queryPoint = {milliVolts, milliAmps, 0};
    if (linInterp2D(&queryPoint) == -1) {
        throw std::invalid_argument("Linear interpolation failed: ("+std::to_string(milliVolts)+", "+std::to_string(milliAmps)+")");
    }
    std::cout << std::to_string(queryPoint.milliAmpHours) << std::flush;
}