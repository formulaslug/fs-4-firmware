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
	bool bmsFaultStatus;
	bool imdStatus;
	bool shutDownCircuitReading;
	bool shutDownIn;
	bool shutDownOut;
	bool preChargeActive;
	bool prechargeDone;
	bool chargeStat;

	bool balnceStat;
	bool cellTooLow;
	bool cellTooHigh;
	bool tempTooLow;
	bool tempTooHigh;
	bool tempTooHighCRG;
	uint8_t faultModIndex;
	uint8_t faultSenseIndex;
	uint8_t battStatFaultIndex; // this is the cell fault num
	uint16_t glvVoltage;
	uint8_t pwmFanstat;


	uint8_t data[8];
	

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
