#include "tempCan.h"
#include "BMS.h"
#include <cstdint>
#include <cstdio>

CanGenerator::CanGenerator(BMS* GivenBMSObject){
	BMSInstance = GivenBMSObject;
}

CANMessage CanGenerator::BuildVoltageMessage(uint8_t modNum) {
    // builds a voltage message
    char data[6];
    for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
        data[j] = (uint8_t)(BMSInstance->voltages[modNum][j] / 10 - 200);
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
            data[i] = BMSInstance->cellTemps[messageIdindex][i];
        }
    } else {
        messageIdindex = (modNum * 2) - 1;
        uint8_t index = 0;
        for (uint8_t i = NUM_TEMP_SENSORS_PER_MODULE / 2; i < NUM_TEMP_SENSORS_PER_MODULE; i++) {
            data[index] = BMSInstance->cellTemps[messageIdindex][i];
            index++;
        }
    }
    return CANMessage{temperatureMessageIds[messageIdindex], data, NUM_TEMP_SENSORS_PER_MODULE / 2};
}

CANMessage CanGenerator::BuildStatusMessage() {
	//data in order according to the dbc

	uint8_t data[8] = {0};

    data[0] = (BMSInstance->Data.bmsFaultStatus) + (BMSInstance->Data.imdStatus<<1) + (BMSInstance->Data.shutDownCircuitReading<<2) + (BMSInstance->Data.shutDownIn << 3)+ (BMSInstance->Data.shutDownOut<<4)+(BMSInstance->Data.preChargeActive << 5) + (BMSInstance->Data.prechargeDone << 6) + (BMSInstance->Data.chargeStat << 7);  
	

    data[1] = BMSInstance->Data.balanceStat + (BMSInstance->Data.cellTooLow << 1) + (BMSInstance->Data.cellTooHigh << 2) + (BMSInstance->Data.tempTooLow<<3)+
    (BMSInstance->Data.tempTooHigh << 4) + (BMSInstance->Data.tempTooHighCRG << 5);

    data[2] = BMSInstance->Data.faultModIndex;
    data[3] = BMSInstance->Data.faultSenseIndex;
    data[4] = BMSInstance->Data.battStatFaultIndex;
    data[5] = (uint8_t)(BMSInstance->Data.glvVoltage >> 8);
    data[6] = (uint8_t)(BMSInstance->Data.glvVoltage);
    data[7] = BMSInstance->Data.pwmFanstat;
    return CANMessage{BATT_TPDO_STATUS, data, 8};

}

CANMessage CanGenerator::BuildTrayTempMessage(uint8_t traytempsensors[5]) {
    char data[5];
    for (uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS; i++) {
        data[i] = (uint8_t)(traytempsensors[i]);
    }

    return CANMessage{BATT_TPDO_TRAY_TEMPS, data, NUM_TRAY_TEMP_SENSORS};
}


void CanGenerator::BuildAndSendMessages() {
    CANMessage gmsg;
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        gmsg = BuildVoltageMessage(i);
        BMSInstance->CAN_POWERTRAIN.write(gmsg);
        gmsg = BuildTempMessage(i, true);
        BMSInstance->CAN_POWERTRAIN.write(gmsg);
        gmsg = BuildTempMessage(i, false);
        BMSInstance->CAN_POWERTRAIN.write(gmsg);
    }
}
