#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "can.h"
#include "mbed.h"

class Telemetry {

public:
    void sendCellVoltages();

    struct status_msg { // this struct needs to be updated to match what were actually sending -
        bool bmsFault;
        bool imdFault;
        bool shutdownFinal;
        bool shutdownIn;
        bool shutdownOut;
        bool precharging;
        bool prechargedone;
        bool charging;
        bool balancing;
        bool cell_too_low;
        bool cell_too_high;
        bool temp_too_low;
        bool temp_too_high;
        bool temp_too_high_charging;
        uint16_t glv_voltage;
        int8_t fault_index;
        int8_t module_fault_index;
        int8_t temp_sensor_fault_index;
    };

    struct tray_temps_msg { // this one too needs to be updated - look into 1 wire bus
        uint8_t temp_busbar;
        uint8_t temp_pack_fuse;
        uint8_t temp_bolted_connection;
    };

    struct powerPerformanceData {
        uint16_t packVoltage;
        uint16_t packCurrent;
        uint8_t Soc;
        uint8_t fanPWM;
        uint32_t instantPwr;
    };

    struct ThermalStats {
        // not totally sure how to implement this it involves sensor data the BMS doesnt have access
        // to given we have a seperate cell temperature message maybe we dont have to? ill keep this
        // here for now
        int8_t maxCellTemp;
        int8_t minCellTemp;
        int8_t avgCellTemp;
        int8_t boltedConnectionTemp;
        int8_t busBarTemp;
        int8_t packFuseTemp;
        int8_t intake_air_temp;
        int8_t cowling_exhaustTemp;
    };

    struct cellVoltages { // i dont know exactly how to pack this
        uint16_t module1Volts;
        uint16_t module2Volts;
        uint16_t module3Volts;
        uint16_t module4Volts;
        uint16_t module5Volts;
    };

    struct cellTemperatures {
        int8_t modlule1TempsA;
        int8_t modlule1TempsB;
        int8_t molduelATempsB;
        int8_t molduel2TempsB;
        int8_t modlule3TempsA;
        int8_t molduel3TempsB;
        int8_t modlule4TempsA;
        int8_t molduel4TempsB;
        int8_t modlule5TempsA;
        int8_t molduel5TempsB;
    };

    cellVoltages voltages;
};

#endif