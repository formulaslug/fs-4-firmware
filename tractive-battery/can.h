#pragma once

#include "BMS.h"
#include "mbed.h"
#include <cstdint>

// struct TelemetryInfo {
//     bool bmsFaultStatus;
//     bool imdStatus;
//     bool shutdownIn;
//     bool shutdownOut;
//     bool shutdownFinal;
//     bool preChargeActive;
//     bool prechargeDone;
//     bool charging;
//
//     bool balanceStat;
//     bool cellTooLow;
//     bool cellTooHigh;
//     bool tempTooLow;
//     bool tempTooHigh;
//     bool tempTooHighCRG;
//     uint8_t faultModIndex;
//     uint8_t faultSenseIndex;
//     uint8_t battStatFaultIndex; // this is the cell fault num
//     uint16_t glvVoltage;
//     uint8_t pwmFanstat;
//     uint16_t socEstimate;
// };
//

// voltage messages
constexpr uint32_t BATT_TPDO_MOD0_VOLTS = 0x494;
constexpr uint32_t BATT_TPDO_MOD1_VOLTS = 0x497;
constexpr uint32_t BATT_TPDO_MOD2_VOLTS = 0x49a;
constexpr uint32_t BATT_TPDO_MOD3_VOLTS = 0x49d;
constexpr uint32_t BATT_TPDO_MOD4_VOLTS = 0x4a0;

// general status message ids
constexpr uint32_t BATT_TPDO_STATUS = 0x391;
constexpr uint32_t BATT_TPDO_POWER = 0x392;
constexpr uint32_t BATT_TPDO_CELL_STATS = 0x393;
constexpr uint32_t BATT_TPDO_TRAY_TEMPS = 0x4c0;

// temperature messages ids
constexpr uint32_t BATT_TPDO_MOD0_TEMPSA = 0x495;
constexpr uint32_t BATT_TPDO_MOD0_TEMPSB = 0x496;
constexpr uint32_t BATT_TPDO_MOD1_TEMPSA = 0x498;
constexpr uint32_t BATT_TPDO_MOD1_TEMPSB = 0x499;
constexpr uint32_t BATT_TPDO_MOD2_TEMPSA = 0x49b;
constexpr uint32_t BATT_TPDO_MOD2_TEMPSB = 0x49c;
constexpr uint32_t BATT_TPDO_MOD3_TEMPSA = 0x49e;
constexpr uint32_t BATT_TPDO_MOD3_TEMPSB = 0x49f;
constexpr uint32_t BATT_TPDO_MOD4_TEMPSA = 0x4a1;
constexpr uint32_t BATT_TPDO_MOD4_TEMPSB = 0x4a2;

constexpr uint32_t VOLTAGE_MESSAGE_IDS[NUM_BATTERY_MODULES] = {
    BATT_TPDO_MOD0_VOLTS,
    BATT_TPDO_MOD1_VOLTS,
    BATT_TPDO_MOD2_VOLTS,
    BATT_TPDO_MOD3_VOLTS,
    BATT_TPDO_MOD4_VOLTS
};
constexpr uint32_t TEMPERATURE_MESSAGE_IDS[NUM_BATTERY_MODULES * 2] = {
    BATT_TPDO_MOD0_TEMPSA,
    BATT_TPDO_MOD0_TEMPSB,
    BATT_TPDO_MOD1_TEMPSA,
    BATT_TPDO_MOD1_TEMPSB,
    BATT_TPDO_MOD2_TEMPSA,
    BATT_TPDO_MOD2_TEMPSB,
    BATT_TPDO_MOD3_TEMPSA,
    BATT_TPDO_MOD3_TEMPSB,
    BATT_TPDO_MOD4_TEMPSA,
    BATT_TPDO_MOD4_TEMPSB
};

namespace CanGenerator {

CANMessage BuildStatusMessage(
    BMS& bms,
    bool imdFault,
    bool shutdownFinal,
    bool shutdownIn,
    bool shutdownOut,
    bool precharging,
    bool prechargeDone,
    uint16_t glvVoltageMv,
    float fanPwmDuty
);
CANMessage BuildTempMessage(BMS& bms, uint8_t modNum, bool AorB);
CANMessage BuildVoltageMessage(BMS& bms, uint8_t modNum);
CANMessage BuildCellStatsMessage(BMS& bms);
CANMessage BuildPowerMessage(BMS& bms, uint16_t socEstimate);
CANMessage BuildTrayTempMessage(uint8_t traytempsensors[5]);

}; // namespace CanGenerator
