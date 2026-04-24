#include "tempCan.h"
#include "BMS.h"
#include <cstdint>
#include <cstdio>

CANMessage CanGenerator::BuildVoltageMessage(uint8_t modNum) {
    // builds a voltage message
    char data[6];
    for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
        data[j] = (uint8_t)(BMSInstance.voltages[modNum][j] / 10 - 200);
    }
    return CANMessage{voltageMessageIds[modNum], data, NUM_VOLTAGES_PER_MODULE};
}

CANMessage CanGenerator::BuildTempMessage(uint8_t modNum, bool AorB) {
    char data[8];
    uint8_t messageIdindex;
    uint8_t limit;
    if (AorB) {
        messageIdindex = modNum * 2;
        for (uint8_t i = 0; i < NUM_TEMP_SENSORS_PER_MODULE / 2; i++) {
            data[i] = BMSInstance.cellTemps[messageIdindex][i];
        }
    } else {
        messageIdindex = (modNum * 2) - 1;
        uint8_t index = 0;
        for (uint8_t i = NUM_TEMP_SENSORS_PER_MODULE / 2; i < NUM_TEMP_SENSORS_PER_MODULE; i++) {
            data[index] = BMSInstance.cellTemps[messageIdindex][i];
            index++;
        }
    }
    return CANMessage{temperatureMessageIds[messageIdindex], data, NUM_TEMP_SENSORS_PER_MODULE / 2};
}

CANMessage CanGenerator::BuildStatusMessage() {}

void CanGenerator::BuildAndSendMessages() {
    CANMessage gmsg;
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        gmsg = BuildVoltageMessage(i);
        BMSInstance.CAN_POWERTRAIN.write(gmsg);
        gmsg = BuildTempMessage(i, true);
        BMSInstance.CAN_POWERTRAIN.write(gmsg);
        gmsg = BuildTempMessage(i, false);
        BMSInstance.CAN_POWERTRAIN.write(gmsg);
    }
}
