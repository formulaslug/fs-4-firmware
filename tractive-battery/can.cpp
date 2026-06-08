#include "can.h"
#include "BMS.h"
#include <cstdint>
#include <cstdio>

namespace CanGenerator {

CANMessage BuildVoltageMessage(BMS& bms, uint8_t modNum) {
    // builds a voltage message
    char data[6] = {0};
    for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
        data[j] = (uint8_t)(bms.voltages[modNum][j] / 10 - 200);
    }
    return CANMessage{VOLTAGE_MESSAGE_IDS[modNum], data, NUM_VOLTAGES_PER_MODULE};
}

CANMessage BuildTempMessage(BMS& bms, uint8_t modNum, bool AorB) {
    char data[6] = {0};
    uint8_t messageIdindex;
    // our temps are in float the scale is 0.25 we need quarters of a degree
    if (AorB) {
        messageIdindex = modNum * 2;
        for (uint8_t i = 0; i < NUM_TEMP_SENSORS_PER_MODULE / 2; i++) {
            data[i] = (uint8_t)(bms.temps[modNum][i] / 0.25f);
        }
    } else {
        messageIdindex = modNum * 2 + 1;
        uint8_t index = 0;
        for (uint8_t i = NUM_TEMP_SENSORS_PER_MODULE / 2; i < NUM_TEMP_SENSORS_PER_MODULE; i++) {
            data[index] = (uint8_t)(bms.temps[modNum][i] / 0.25f);
            index++;
        }
    }
    return CANMessage{
        TEMPERATURE_MESSAGE_IDS[messageIdindex], data, NUM_TEMP_SENSORS_PER_MODULE / 2
    };
}

/*
   BATT_STATUS_IMD_FAULT
   BATT_STATUS_SHUTDOWN_FINAL
   BATT_STATUS_SHUTDOWN_IN
   BATT_STATUS_SHUTDOWN_OUT
   BATT_STATUS_PRECHARGING
   BATT_STATUS_PRECHARGE_DONE
   BATT_STATUS_CHARGING
   BATT_STATUS_BALANCING
   BATT_STATUS_CELL_VOLTAGE_TOO_LOW
   BATT_STATUS_CELL_VOLTAGE_TOO_HIGH
   BATT_STATUS_PACK_TEMP_TOO_LOW
   BATT_STATUS_PACK_TEMP_TOO_HIGH
   BATT_STATUS_PACK_TEMP_TOO_HIGH_CRG
   BATT_STATUS_FAULT_MODULE_INDEX
   BATT_STATUS_FAULT_SENSOR_INDEX
   BATT_STATUS_FAULT_INDEX
   BATT_STATUS_GLV_VOLTAGE
   BATT_STATUS_FAN_PWM_DUTY
*/
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
) {
    uint8_t data[8] = {0};

    data[0] = ((bms.currentState == BMS::FAULT) << 0)
              | (imdFault << 1)
              | (shutdownFinal << 2)
              | (shutdownIn << 3)
              | (shutdownOut << 4)
              | (precharging << 5)
              | (prechargeDone << 6)
              | ((bms.currentState == BMS::CHARGING) << 7);

    data[1] = (bms.balancing << 0)
              | (bms.cellVoltageTooLow << 1)
              | (bms.cellVoltageTooHigh << 2)
              | (bms.packTempTooLow << 3)
              | (bms.packTempTooHigh << 4)
              | (bms.packTempTooHighCrg << 5);

    // we need to add updates here
    data[2] = bms.faultModuleIndex;
    data[3] = bms.faultSensorIndex;
    data[4] = bms.faultModuleIndex * NUM_BATTERY_MODULES + bms.faultSensorIndex;
    // GLV voltage, scale = 0.001, low byte first
    data[5] = (uint8_t)(glvVoltageMv & 0xFF);
    data[6] = (uint8_t)((glvVoltageMv >> 8) & 0x00FF);
    data[7] = fanPwmDuty * 100;
    return CANMessage{BATT_TPDO_STATUS, data, 8};
}

CANMessage BuildCellStatsMessage(BMS& bms) {
    uint8_t data[6] = {0};

    int32_t sumTemp = 0;
    int8_t minTemp = bms.temps[0][0];
    int8_t maxTemp = bms.temps[0][0];
    uint16_t tempCount = 0;

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            int8_t t = bms.temps[i][j];
            sumTemp += t;
            if (t < minTemp) minTemp = t;
            if (t > maxTemp) maxTemp = t;
            tempCount++;
        }
    }
    float avgTemp = (float)sumTemp / tempCount;
    int32_t sumVolt = 0;
    uint16_t minVolt = bms.voltages[0][0];
    uint16_t maxVolt = bms.voltages[0][0];
    uint16_t voltCount = 0;

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
            uint16_t v = bms.voltages[i][j];
            sumVolt += v;
            if (v < minVolt) minVolt = v;
            if (v > maxVolt) maxVolt = v;
            voltCount++;
        }
    }
    float avgVoltMv = (float)sumVolt / voltCount;
    data[0] = (uint8_t)(avgTemp / 0.25f);
    data[1] = (uint8_t)(maxTemp / 0.25f);
    data[2] = (uint8_t)(minTemp / 0.25f);

    // volts: scale=0.01, bias=2, raw =s mV/10 - 200
    data[3] = (uint8_t)((avgVoltMv / 10.0f) - 200.0f);
    data[4] = (uint8_t)((maxVolt / 10) - 200);
    data[5] = (uint8_t)((minVolt / 10) - 200);
    return CANMessage{BATT_TPDO_CELL_STATS, data, 6};
}

CANMessage BuildPowerMessage(BMS& bms, uint16_t socEstimate) {
    uint8_t data[8] = {0};
    uint16_t packVolts = (uint16_t)(bms.packVoltageMv / 10);
    // scale of 0.1

    data[0] = (uint8_t)(packVolts >> 8);
    data[1] = (uint8_t)(packVolts);

    int16_t curr = (uint16_t)(bms.packCurrent * 10);
    data[2] = (uint8_t)(curr >> 8);
    data[3] = (uint8_t)(curr);

    // again scale of 0.1

    int32_t totalPower = bms.packCurrent * bms.packVoltageMv / 1000.0f;
    uint8_t top2bits = (uint8_t)((totalPower & 0x00300000) << 8);
    data[4] = (uint8_t)(totalPower << 24);
    data[5] = (uint8_t)(totalPower << 16);
    data[6] |= top2bits;

    // state of charge remains to be done ...

    uint16_t socEstimateShift = socEstimate;
    socEstimateShift &= 0xFFC0;
    // isolate the top 10 bits
    socEstimateShift = socEstimateShift >> 2;
    // shift it over in accordance with this cursed can message
    data[6] |= (uint8_t)(socEstimateShift & 0x3f);
    data[7] |= (int8_t)(socEstimateShift & 0x00f);

    return CANMessage{BATT_TPDO_POWER, data, 8};
}

CANMessage BuildTrayTempMessage(uint8_t traytempsensors[5]) {
    char data[5] = {0};
    for (uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS; i++) {
        uint8_t trayTempData = (uint8_t)traytempsensors[i] * 2;
        // tray temp messages have a scale of 0.5
        data[i] = trayTempData;
    }

    return CANMessage{BATT_TPDO_TRAY_TEMPS, data, NUM_TRAY_TEMP_SENSORS};
}

} // namespace CanGenerator
