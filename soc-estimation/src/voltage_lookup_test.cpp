#include "voltage_lookup.h"
#include "batteryLUT.h"
#include <iostream>
#include <string>

#define MESSAGE(X) std::cout << X << "\n"

int main(int argc, char const* argv[]) {
    // test discharge data point printout
    MESSAGE("Data Print Out");
    std::cout << DISCHARGE_CAPACITY_LUT[0][0].toString() << "\n";
    // test amperage lower bound
    MESSAGE("Amperage Lower Bound");
    std::cout << getAmperageLowerBoundIndex(-1) << "\n";
    std::cout << DISCHARGE_CAPACITY_LUT[0][getAmperageLowerBoundIndex(700)].toString() << "\n";
    std::cout << DISCHARGE_CAPACITY_LUT[0][getAmperageLowerBoundIndex(1500)].toString() << "\n";
    // test voltage lower bound
    MESSAGE("Voltage Lower Bound");
    std::cout << getVoltageLowerBoundIndex(-1) << "\n";
    std::cout << DISCHARGE_CAPACITY_LUT[getVoltageLowerBoundIndex(2600)][0].toString() << "\n";
    std::cout << DISCHARGE_CAPACITY_LUT[getVoltageLowerBoundIndex(2800)][0].toString() << "\n";
    // test amp lin interp
    MESSAGE("Voltage Lin Interp");
    std::cout << linInterpCapacityVoltage(4, DISCHARGE_CAPACITY_LUT[0][0], DISCHARGE_CAPACITY_LUT[1][0]) << "\n";
    std::cout << linInterpCapacityVoltage(0, DISCHARGE_CAPACITY_LUT[0][0], DISCHARGE_CAPACITY_LUT[1][0]) << "\n";
    std::cout << linInterpCapacityVoltage(8, DISCHARGE_CAPACITY_LUT[0][0], DISCHARGE_CAPACITY_LUT[1][0]) << "\n";
    // test amp lin interp
    MESSAGE("2d Lin Interp");
    DischargeDataPoint testPoint = {4,5,0};
    linInterp2D(&testPoint);
    float test = 1.2;
    std::cout << testPoint.milliAmpHours << "\n";
    return 1;
}