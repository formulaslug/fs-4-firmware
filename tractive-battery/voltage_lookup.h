#ifndef VOLTAGE_LOOKUP
#define VOLTAGE_LOOKUP

// Constants & Includes
#include <string>

// convert to battery percent
// convert to energy?

// Structs
struct DischargeDataPoint {
    int milliVolts;
    int milliAmps;
    float milliAmpHours;

    std::string toString() const
    {
        return "(" + std::to_string(milliVolts) + ", " + std::to_string(milliAmps) + ", " + std::to_string(milliAmpHours) + ")";
    }
};

// Function Prototypes
int getAmperageLowerBoundIndex(int milliAmps);
int getVoltageLowerBoundIndex(int milliVolts);

float linInterpCapacityVoltage(int milliVolts, DischargeDataPoint lower, DischargeDataPoint upper);
float linInterpCapacityAmperage(int milliAmps, DischargeDataPoint lower, DischargeDataPoint upper);
int linInterp2D(DischargeDataPoint *batteryCharacteristics);

#endif