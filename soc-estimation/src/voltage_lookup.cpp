#include "voltage_lookup.h"
#include "batteryLUT.h"

int getAmperageLowerBoundIndex(int milliAmps) {
    // check if under lowest value
    if (DISCHARGE_CAPACITY_LUT[0][0].milliAmps > milliAmps) {
        return -1;
    }

    int lowestAmperageIndex = 0;
    for (; lowestAmperageIndex < LUT_AMPERAGE_ENTRIES; lowestAmperageIndex++) {
        if (DISCHARGE_CAPACITY_LUT[0][lowestAmperageIndex].milliAmps >= milliAmps) {
            break;
        }
    }
    return lowestAmperageIndex - 1;
}

int getVoltageLowerBoundIndex(int milliVolts) {
    // check if under lowest value
    if (DISCHARGE_CAPACITY_LUT[0][0].milliVolts > milliVolts) {
        return -1;
    }

    int lowestVoltageIndex = 0;
    for (; lowestVoltageIndex < LUT_AMPERAGE_ENTRIES; lowestVoltageIndex++) {
        if (DISCHARGE_CAPACITY_LUT[lowestVoltageIndex][0].milliVolts >= milliVolts) {
            break;
        }
    }
    return lowestVoltageIndex - 1;
}

// imp out of bounds check
float linInterpCapacityVoltage(int milliVolts, DischargeDataPoint lower, DischargeDataPoint upper) {
    float voltageDifference = upper.milliVolts - lower.milliVolts;
    float capacityDifference = upper.milliAmpHours - lower.milliAmpHours;
    float voltageAboveLower = milliVolts - lower.milliVolts;
    return lower.milliAmpHours + (voltageAboveLower * (capacityDifference / voltageDifference));
}

// imp out of bounds check
float linInterpCapacityAmperage(int milliAmps, DischargeDataPoint lower, DischargeDataPoint upper) {
    float amperageDifference = upper.milliAmps - lower.milliAmps;
    float capacityDifference = upper.milliAmpHours - lower.milliAmpHours;
    float amperageAboveLower = milliAmps - lower.milliAmps;
    return lower.milliAmpHours + (amperageAboveLower * (capacityDifference / amperageDifference));
}

int linInterp2D(DischargeDataPoint* batteryCharacteristics) {
    // get upper and lower bounds of current and voltage point
    int amperageLB = getAmperageLowerBoundIndex(batteryCharacteristics->milliAmps);
    int voltageLB = getVoltageLowerBoundIndex(batteryCharacteristics->milliVolts);
    int amperageUB = amperageLB + 1;
    int voltageUB = voltageLB + 1;

    // check if in LUT Bounds
    if (amperageLB == -1 || voltageLB == -1) {
        return -1;
    }

    // linear interpolate along amperage
    float lowerVoltageCapacity = linInterpCapacityAmperage(batteryCharacteristics->milliAmps, DISCHARGE_CAPACITY_LUT[voltageLB][amperageLB], DISCHARGE_CAPACITY_LUT[voltageLB][amperageUB]);
    float upperVoltageCapacity = linInterpCapacityAmperage(batteryCharacteristics->milliAmps, DISCHARGE_CAPACITY_LUT[voltageUB][amperageLB], DISCHARGE_CAPACITY_LUT[voltageUB][amperageUB]);

    // normal procedure
    DischargeDataPoint lowerVoltagePoint = {DISCHARGE_CAPACITY_LUT[voltageLB][amperageLB].milliVolts, batteryCharacteristics->milliAmps, lowerVoltageCapacity};
    DischargeDataPoint upperVoltagePoint = {DISCHARGE_CAPACITY_LUT[voltageUB][amperageLB].milliVolts, batteryCharacteristics->milliAmps, upperVoltageCapacity};
    batteryCharacteristics->milliAmpHours = linInterpCapacityVoltage(batteryCharacteristics->milliVolts, lowerVoltagePoint, upperVoltagePoint);
    return 1;
}