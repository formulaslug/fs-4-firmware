#include "tempCan.h"
#include "BMS.h"
#include <cstdint>
#include <cstdio>

CanGenerator::CanGenerator(const BMS& GivenBMSObject, CAN& CAN_POWERTRAIN)
    : BMSInstance(GivenBMSObject), CAN_POWERTRAIN(CAN_POWERTRAIN) {}

CANMessage CanGenerator::BuildVoltageMessage(uint8_t modNum) {
    // builds a voltage message
    char data[6] = {0};
    for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
        data[j] = (uint8_t)(BMSInstance.voltages[modNum][j] / 10 - 200);
    }
    return CANMessage{VOLTAGE_MESSAGE_IDS[modNum], data, NUM_VOLTAGES_PER_MODULE};
}

CANMessage CanGenerator::BuildTempMessage(uint8_t modNum, bool AorB) {
    char data[8] = {0};
    uint8_t messageIdindex;
    // uint8_t limit;
    if (AorB) {
        messageIdindex = modNum * 2;
        for (uint8_t i = 0; i < NUM_TEMP_SENSORS_PER_MODULE / 2; i++) {
            data[i] = BMSInstance.temps[messageIdindex][i];
        }
    } else {
        messageIdindex = (modNum * 2) - 1;
        uint8_t index = 0;
        for (uint8_t i = NUM_TEMP_SENSORS_PER_MODULE / 2; i < NUM_TEMP_SENSORS_PER_MODULE; i++) {
            data[index] = BMSInstance.temps[messageIdindex][i];
            index++;
        }
    }
    return CANMessage{
        TEMPERATURE_MESSAGE_IDS[messageIdindex], data, NUM_TEMP_SENSORS_PER_MODULE / 2
    };
}

CANMessage CanGenerator::BuildStatusMessage() {
    uint8_t data[8] = {0};

    data[0] = (BMSInstance.Data.bmsFaultStatus)
              + (BMSInstance.Data.imdStatus << 1)
              + (BMSInstance.Data.shutdownFinal << 2)
              + (BMSInstance.Data.shutdownIn << 3)
              + (BMSInstance.Data.shutdownOut << 4)
              + (BMSInstance.Data.preChargeActive << 5)
              + (BMSInstance.Data.prechargeDone << 6)
              + (BMSInstance.Data.charging << 7);

    data[1] = BMSInstance.Data.balanceStat
              + (BMSInstance.Data.cellTooLow << 1)
              + (BMSInstance.Data.cellTooHigh << 2)
              + (BMSInstance.Data.tempTooLow << 3)
              + (BMSInstance.Data.tempTooHigh << 4)
              + (BMSInstance.Data.tempTooHighCRG << 5);

    data[2] = BMSInstance.Data.faultModIndex;
    data[3] = BMSInstance.Data.faultSenseIndex;
    data[4] = BMSInstance.Data.battStatFaultIndex;
    data[5] = (uint8_t)(BMSInstance.Data.glvVoltage >> 8);
    data[6] = (uint8_t)(BMSInstance.Data.glvVoltage);
    data[7] = BMSInstance.Data.pwmFanstat;
    return CANMessage{BATT_TPDO_STATUS, data, 8};
}

CANMessage CanGenerator::BuildTrayTempMessage(uint8_t traytempsensors[5]) {
    char data[5] = {0};
    for (uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS; i++) {
        data[i] = (uint8_t)(traytempsensors[i]);
    }

    return CANMessage{BATT_TPDO_TRAY_TEMPS, data, NUM_TRAY_TEMP_SENSORS};
}

CANMessage CanGenerator::BuildCellStatsMessage() {
    uint8_t data[5] = {0};

    int32_t sumTemp = 0;
    int8_t minTemp = BMSInstance.temps[0][0];
    int8_t maxTemp = BMSInstance.temps[0][0];
    uint16_t tempCount = 0;

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            int8_t t = BMSInstance.temps[i][j];
            sumTemp += t;
            if (t < minTemp) minTemp = t;
            if (t > maxTemp) maxTemp = t;
            tempCount++;
        }
    }
    float avgTemp = (float)sumTemp / tempCount;
    int32_t sumVolt = 0;
    uint16_t minVolt = BMSInstance.voltages[0][0];
    uint16_t maxVolt = BMSInstance.voltages[0][0];
    uint16_t voltCount = 0;

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
            uint16_t v = BMSInstance.voltages[i][j];
            sumVolt += v;
            if (v < minVolt) minVolt = v;
            if (v > maxVolt) maxVolt = v;
            voltCount++;
        }
    }
    float avgVolt = (float)sumVolt / voltCount;
    data[0] = (uint8_t)(avgTemp / 0.25f);
    data[1] = (uint8_t)(maxTemp / 0.25f);
    data[2] = (uint8_t)(minTemp / 0.25f);
    data[3] = (uint8_t)(avgVolt / 0.25f);
    data[4] = (uint8_t)(maxVolt / 0.25f);
    data[5] = (uint8_t)(minVolt / 0.25f);
    return CANMessage{BATT_TPDO_CELL_STATS, data, 6};
}

void CanGenerator::BuildAndSendMessages() {
    CANMessage msg;
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        msg = BuildVoltageMessage(i);
        CAN_POWERTRAIN.write(msg);
        msg = BuildTempMessage(i, true);
        CAN_POWERTRAIN.write(msg);
        msg = BuildTempMessage(i, false);
        CAN_POWERTRAIN.write(msg);
    }
}
