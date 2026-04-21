#include "state_of_health.h"
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char const* argv[]) {
    // arg1: last SOH
    // arg2: current draw
    // arg3: millis since last call
    // outputs: SOH at current time
    if (argc != 4) {
        throw std::invalid_argument("Expected 3 arguments, got: " + std::to_string(argc - 1));
    }

    // parse input as floats
    float lastSOH;
    float currentDraw;
    float milliSinceLastCall;
    try {
        lastSOH = std::stod(argv[1]);
        currentDraw = std::stod(argv[2]);
        milliSinceLastCall = std::stod(argv[3]);
    } catch (...) {
        throw std::invalid_argument("Expected three floats");
    }

    // feed to SOH func
    float SOH = sohEstimate(lastSOH,currentDraw,milliSinceLastCall);
    
    std::cout << std::to_string(SOH) << std::flush;
}