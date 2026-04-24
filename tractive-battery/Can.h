#ifndef _FS4_BMS_CAN_H_
#define _FS4_BMS_CAN_H_

#include "mbed.h"
#include <cstdint>
// #include "BmsConfig.h"

// BMS to Bus
//  Global pack status
extern BMS BMSInstance;

constexpr uint32_t kID_STATUS = 0x188;

// Power performance data
constexpr uint32_t kID_POWER = 0x288;

// Thermal stats (max/min/avg temps, tray sensors)
constexpr uint32_t kID_THERMAL = 0x388;

// Per-module cell voltages:
constexpr uint32_t kMOD_VOLTS_A_IDS[NUM_BATTERY_MODULES] = {0x191, 0x192, 0x193, 0x194, 0x195};
constexpr uint32_t kMOD_VOLTS_B_IDS[NUM_BATTERY_MODULES] = {0x1A1, 0x1A2, 0x1A3, 0x1A4, 0x1A5};

// Per-module cell temperatures:
constexpr uint32_t kMOD_TEMPS_A_IDS[NUM_BATTERY_MODULES] = {0x291, 0x292, 0x293, 0x294, 0x295};
constexpr uint32_t kMOD_TEMPS_B_IDS[NUM_BATTERY_MODULES] = {0x2A1, 0x2A2, 0x2A3, 0x2A4, 0x2A5};

constexpr uint32_t kID_HEARTBEAT = 0x703;

// Bus to BMS
constexpr uint32_t kID_MC_TEMP = 0x682;
constexpr uint32_t kID_CHARGER_STATUS = 0x190;
constexpr uint32_t kID_MAX_CURRENTS = 0x286;

CANMessage buildStatusMsg(const BMS::status_msg& s);

CANMessage buildPowerMsg(
    uint16_t packVoltage_mV,
    int16_t packCurrent_mA,
    uint8_t soc_percent,
    uint8_t fanPWM_percent,
    int32_t instPower_W
);

CANMessage buildThermalMsg(
    int8_t maxCellTemp,
    int8_t minCellTemp,
    int8_t avgCellTemp,
    int8_t boltedConnTemp,
    int8_t busBarTemp,
    int8_t packFuseTemp,
    int8_t intakeAirTemp,
    int8_t cowlingExhaustTemp
);

// Module voltage messages
CANMessage buildModuleVoltsA(uint32_t canId, const uint16_t* moduleVolts); // cells 0–3
CANMessage buildModuleVoltsB(uint32_t canId, const uint16_t* moduleVolts); // cells 4–5

// Module temperature messages
CANMessage buildModuleTempsA(uint32_t canId, const int8_t* moduleTemps); // sensors  0–7
CANMessage buildModuleTempsB(uint32_t canId, const int8_t* moduleTemps); // sensors 8–11

void canSendAll(BMS& bms);

void canRead(BMS& bms);

#endif
