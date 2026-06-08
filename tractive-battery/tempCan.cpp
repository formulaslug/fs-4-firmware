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
    char data[6] = {0};
    uint8_t messageIdindex;
    // our temps are in float the scale is 0.25 we need quarters of a degree 
        if (AorB) {
        messageIdindex = modNum * 2;
        for (uint8_t i = 0; i < NUM_TEMP_SENSORS_PER_MODULE / 2; i++) {
            data[i] = (uint8_t)(BMSInstance.temps[modNum][i] / 0.25f);
        }
    } else {
        messageIdindex = modNum * 2 + 1;
        uint8_t index = 0;
        for (uint8_t i = NUM_TEMP_SENSORS_PER_MODULE / 2; i < NUM_TEMP_SENSORS_PER_MODULE; i++) {
            data[index] = (uint8_t)(BMSInstance.temps[modNum][i] / 0.25f);
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
    //GLV voltage, scale = 0.001, low byte first
    data[5] = (uint8_t)(Data.glvVoltage & 0x00FF);
    data[6] = (uint8_t)((Data.glvVoltage >> 8) & 0x00FF);
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

CANMessage CanGenerator::BuildPowerMessage(TelemetryInfo &Data){
    uint8_t data[8] = {0};
    uint16_t packVolts = (uint16_t)(BMSInstance.packVoltageMv/10);
    //scale of 0.1

    data[0] = (uint8_t)(packVolts>>8); 
    data[1] = (uint8_t)(packVolts);
    
    int16_t curr = (uint16_t)(BMSInstance.packCurrent*10);
    data[2] = (uint8_t)(curr>>8);
    data[3] = (uint8_t)(curr);

    //again scale of 0.1 

    int32_t totalPower = BMSInstance.packCurrent * BMSInstance.packVoltageMv/1000.0f; 
    uint8_t top2bits = (uint8_t)((totalPower & 0x00300000)<<8);
    data[4] = (uint8_t)(totalPower << 24);
    data[5] = (uint8_t)(totalPower <<16);
    data[6] |= top2bits;


    //state of charge remains to be done ... 
    
    uint16_t socEstimateShift = Data.socEstimate;
    socEstimateShift &=  0xFFC0;
    //isolate the top 10 bits
    socEstimateShift  = socEstimateShift >> 2;
    //shift it over in accordance with this cursed can message
    data[6] |= (uint8_t)(socEstimateShift & 0x3f);
    data[7] |= (int8_t)(socEstimateShift & 0x00f);

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
    Data.bmsFaultStatus = BMSInstance.currentState == BMS::FAULT;
    Data.imdStatus = BMSInstance.imdFaultStat;
    Data.tempTooLow = BMSInstance.cellTooLow;
    Data.tempTooHigh = BMSInstance.cellTooHigh;
    Data.shutdownIn = Shutdown_In_3V3_Filtered.read();
    Data.shutdownOut = Shutdown_Out_3V3_Filtered.read();
    Data.tempTooHighCRG = ((BMSInstance.currentState == BMS::CHARGING) && BMSInstance.cellTooHigh);
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
        ThisThread::sleep_for(2ms);
        msg = BuildPowerMessage(Data);
        CAN_POWERTRAIN.write(msg);
        ThisThread::sleep_for(2ms);
    }
    msg = BuildStatusMessage(Data);
    CAN_POWERTRAIN.write(msg);
}
