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
    // our temps are in float the scale is 0.25 we need quarters of a degree 
    if (AorB) {
        messageIdindex = modNum * 2;
        for (uint8_t i = 0; i < NUM_TEMP_SENSORS_PER_MODULE / 2; i++) {
            float recordedTemp = BMSInstance.temps[messageIdindex][i];
            recordedTemp*=100;
            uint8_t canTempData = (uint8_t)recordedTemp;
            canTempData *= 4;
            data[i] = canTempData;
        }
    } else {
        messageIdindex = (modNum * 2) - 1;
        uint8_t index = 0;
        for (uint8_t i = NUM_TEMP_SENSORS_PER_MODULE / 2; i < NUM_TEMP_SENSORS_PER_MODULE; i++) {
            // data[index] = BMSInstance.temps[messageIdindex][i];
            float recordedTemp = BMSInstance.temps[messageIdindex][i];
            recordedTemp*=100;
            uint8_t canTempData = (uint8_t)recordedTemp;
            canTempData *= 4;
            data[index]= canTempData;
            index++;
        }
    }
    return CANMessage{
        TEMPERATURE_MESSAGE_IDS[messageIdindex], data, NUM_TEMP_SENSORS_PER_MODULE / 2
    };
}

CANMessage CanGenerator::BuildStatusMessage(TelemetryInfo Data) {
    uint8_t data[8] = {0};

    data[0] = (Data.bmsFaultStatus)
              + (Data.imdStatus << 1)
              + (Data.shutdownFinal << 2)
              + (Data.shutdownIn << 3)
              + (Data.shutdownOut << 4)
              + (Data.preChargeActive << 5)
              + (Data.prechargeDone << 6)
              + (Data.charging << 7);

    data[1] = Data.balanceStat
              + (Data.cellTooLow << 1)
              + (Data.cellTooHigh << 2)
              + (Data.tempTooLow << 3)
              + (Data.tempTooHigh << 4)
              + (Data.tempTooHighCRG << 5);

    // we need to add updates here   
    data[2] = Data.faultModIndex;
    data[3] = Data.faultSenseIndex;
    data[4] = Data.battStatFaultIndex;
    data[5] = (uint8_t)(Data.glvVoltage >> 8);
    data[6] = (uint8_t)(Data.glvVoltage);
    data[7] = Data.pwmFanstat;
    return CANMessage{BATT_TPDO_STATUS, data, 8};
}

CANMessage CanGenerator::BuildTrayTempMessage(uint8_t traytempsensors[5]) {
    char data[5] = {0};
    for (uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS; i++) {
        uint8_t trayTempData = (uint8_t)traytempsensors[i] * 2;
        //tray temp messages have a scale of 0.5
        data[i] = trayTempData;
    }

    return CANMessage{BATT_TPDO_TRAY_TEMPS, data, NUM_TRAY_TEMP_SENSORS};
}

CANMessage CanGenerator::BuildCellStatsMessage() {
    uint8_t data[6] = {0};

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

CANMessage CanGenerator::BuildPowerMessage(){
    uint8_t data[8] = {0};
    uint16_t packVolts = (uint16_t)(BMSInstance.packVoltageMv/10);
    //scale of 0.1

    data[0] = (uint8_t)(packVolts>>8); 
    data[1] = (uint8_t)(packVolts);
    
    int16_t posCurr = (uint16_t)(BMSInstance.packCurrentAmpsOutput*10);
    int16_t negCurr = (uint16_t)(BMSInstance.packCurrentAmpsInput*10) ;
    int16_t totalCurr = posCurr - negCurr;
    data[2] = (uint8_t)(totalCurr>>8);
    data[3] = (uint8_t)(totalCurr);

    //again scale of 0.1 

    int32_t totalPower = totalCurr * packVolts; 
    uint8_t top2bits = (uint8_t)((totalPower & 0x00300000)<<8);
    data[4] = (uint8_t)(totalPower << 24);
    data[5] = (uint8_t)(totalPower <<16);
    data[6] |= top2bits;


    //state of charge remains to be done ... .

    return CANMessage{BATT_TPDO_POWER, data, 8};

  
}


void CanGenerator::updateTelemetry(TelemetryInfo &Data){
    if(BMSInstance.faultLoc == BMS::VOLTAGE){
        Data.faultModIndex = BMSInstance.faultModIndex * NUM_VOLTAGES_PER_MODULE; 
    }else if(BMSInstance.faultLoc == BMS::TEMPS){
        Data.faultModIndex = BMSInstance.faultModIndex * NUM_TRAY_TEMP_SENSORS;
    }else{
        //we default to temperature index in the edge case....
        Data.faultModIndex = BMSInstance.faultModIndex * NUM_TRAY_TEMP_SENSORS;
    }
    Data.faultModIndex = BMSInstance.faultModIndex;
    Data.faultSenseIndex = BMSInstance.faultSenseIndex;
    Data.glvVoltage = BMSInstance.glvVoltage;
    Data.imdStatus = BMSInstance.imdFaultStat;
    Data.tempTooLow = BMSInstance.cellTooLow;
    Data.tempTooHigh = BMSInstance.cellTooHigh;
    Data.shutdownIn = Shutdown_In_3V3_Filtered.read();
    Data.shutdownOut = Shutdown_Out_3V3_Filtered.read();
}


void CanGenerator::BuildAndSendMessages(TelemetryInfo &Data) {
    
    updateTelemetry(Data);

    CANMessage msg;
    
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        msg = BuildVoltageMessage(i);
        CAN_POWERTRAIN.write(msg);
        msg = BuildTempMessage(i, true);
        CAN_POWERTRAIN.write(msg);
        msg = BuildTempMessage(i, false);
        CAN_POWERTRAIN.write(msg);
        msg = BuildPowerMessage();
        CAN_POWERTRAIN.write(msg);
    }
    BuildStatusMessage(Data);
}
