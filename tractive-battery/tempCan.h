#pragma once

#include "BMS.h"
#include "mbed.h"
#include <cstdint>

// voltage messages
constexpr uint32_t BATT_TPDO_MOD0_VOLTS = 0x191;
constexpr uint32_t BATT_TPDO_MOD1_VOLTS = 0x192;
constexpr uint32_t BATT_TPDO_MOD2_VOLTS = 0x193;
constexpr uint32_t BATT_TPDO_MOD3_VOLTS = 0x194;
constexpr uint32_t BATT_TPDO_MOD4_VOLTS = 0x195;

// general status message ids
constexpr uint32_t BATT_TPDO_STATUS = 0x188;
constexpr uint32_t BATT_TPDO_POWER = 0x288;
constexpr uint32_t BATT_TPDO_CELL_STATS = 0x488;
constexpr uint32_t BATT_TPDO_TRAY_TEMPS = 0x388;

// temperature messages ids
constexpr uint32_t BATT_TPDO_MOD0_TEMPSA = 0x291;
constexpr uint32_t BATT_TPDO_MOD0_TEMPSB = 0x292;
constexpr uint32_t BATT_TPDO_MOD1_TEMPSA = 0x293;
constexpr uint32_t BATT_TPDO_MOD1_TEMPSB = 0x294;
constexpr uint32_t BATT_TPDO_MOD2_TEMPSA = 0x295;
constexpr uint32_t BATT_TPDO_MOD2_TEMPSB = 0x296;
constexpr uint32_t BATT_TPDO_MOD3_TEMPSA = 0x297;
constexpr uint32_t BATT_TPDO_MOD3_TEMPSB = 0x298;
constexpr uint32_t BATT_TPDO_MOD4_TEMPSA = 0x299;
constexpr uint32_t BATT_TPDO_MOD4_TEMPSB = 0x29A;

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

class CanGenerator {

private:
    BMS* BMSInstance;

public:
    CanGenerator(BMS*);
    CANMessage BuildTempMessage(uint8_t modNum, bool AorB);
    CANMessage BuildVoltageMessage(uint8_t modNum);
    CANMessage BuildStatusMessage(); // i think this one is likely to be a pain, will probably
                                     // require some changes to the BMS Class.

    CANMessage BuildTrayTempMessage(uint8_t traytempsensors[5]);
    CANMessage BuildCellStatsMessage();
    void BuildAndSendMessages();
};
