#pragma once

#include "DS18B20.h"
#include "LTC6810.h"
#include "LTC681xBus.h"
#include "LTC681xCommand.h"
#include "LTC681xParallelBus.h"
#include "mbed.h"

// general config
inline constexpr uint8_t NUM_BATTERY_MODULES = 5;
inline constexpr uint8_t NUM_VOLTAGES_PER_MODULE = 6;
inline constexpr uint8_t NUM_TEMP_SENSORS_PER_MODULE = 12;
inline constexpr uint8_t NUM_TRAY_TEMP_SENSORS = 5;
inline constexpr uint8_t FAULT_LIMIT = 4; // this is the total number of times we detect a fault state before we actually throw a fault
// okay so ive looked at the kicad and these have not been wired differently so 11/12 sensors have
// the address of 0x48 (based on the data sheet) based on how they are wired
//  i assume this is a wip and will be updated later
inline constexpr uint16_t TMP1075_ADDRESSES[NUM_TEMP_SENSORS_PER_MODULE] = {
    0x48, 0x49, 0x4B, 0x4F, 0x4D, 0x58, 0x4A, 0x4E, 0x40, 0x4C, 0x42, 0x41
};

// this is a temporary thing - i dont know the actual addresses - 64 bit unique address for each
// sensor
inline constexpr uint64_t TRAYTEMP_SENSOR_ADDRESSES[NUM_TRAY_TEMP_SENSORS] = {
    0x860000112ffda728, 0x520000112fffdd28, 0x7400001130aabd28, 0x7400001130aabd28
};

// battery cell info for inr-18650-p30b - based on datasheet
inline constexpr int8_t MAX_CELL_TEMP_CHARGING = 60;
inline constexpr int8_t MIN_CELL_TEMP_CHARGING = 0;
inline constexpr int8_t MAX_CELL_TEMP = 60;
inline constexpr int8_t MIN_CELL_TEMP = -40;

inline constexpr uint16_t MAX_CELL_VOLTAGE_MV = 4200; // 4.2 volts
inline constexpr uint16_t MIN_CELL_VOLTAGE_MV = 2500; // 2.5 volts

// Min cell voltage for balancing while active (mv)
// inline constexpr uint16_t BALANCING_THRESHOLD = 0.85 * MAX_CELL_VOLTAGE; // 0.85*4.2v = 3.57v
inline constexpr uint16_t BALANCING_THRESHOLD = 3000;  
// Min cell difference for balancing (mv)
inline constexpr uint16_t DIFFERENCE_THRESHOLD = 1;

class BMS {

private:
    void chargingActions();
    void turnOnBalancing();
    void readCellVoltages();
    void readTemps();
    void checkForFaults();
    void throwFault(int moduleIndex, int sensorIndex);
    void generateStatusMessage();
    void readPackCurrent();
    void turnOffBalancing();
    void telemetryPins();

public:
    BMS(CAN& CAN_POWERTRAIN, bool charging, Mutex& mainMutex);

    void controller();

    // Thread bmsControllerThread;
    // EventQueue bmsEventQueue;

    struct TMP1075_Handle_t {
        uint8_t i2c_address;
        uint8_t temp_reg;
    };
    enum bms_state { ACTIVE = 0, CHARGING = 1, FAULT = 2 };

    enum fault_location { VOLTAGE = 0, TEMPS = 1, BOTH = 2, NONE = 3 };

    // Current in Amps. Positive for discharge, negative for regen
    float packCurrent;

    Mutex& TelemetryLock; 

    bms_state currentState;

    uint8_t faultCounter = 0;

    bool balancing = false;
    bool cellVoltageTooLow = false;
    bool cellVoltageTooHigh = false;
    bool packTempTooLow = false;
    bool packTempTooHigh = false;
    bool packTempTooHighCrg = false;

    uint8_t faultModuleIndex = 0;
    uint8_t faultSensorIndex = 0;

    Timer ltcTimeoutTimer;
    CANMessage msg;

    // All in mv
    uint16_t voltages[NUM_BATTERY_MODULES][NUM_VOLTAGES_PER_MODULE] = {{0}};
    uint16_t minCellVoltage = 65535;
    uint16_t maxCellVoltage = 0;
    uint32_t packVoltageMv = 0;

    float temps[NUM_BATTERY_MODULES][NUM_TEMP_SENSORS_PER_MODULE] = {{0}};
    uint16_t maxCellTemp;

    std::vector<LTC6810> chips;
    LTC6810::TMP1075_Handle_t tempSensors[NUM_BATTERY_MODULES][NUM_TEMP_SENSORS_PER_MODULE];

    // pin definitions
    // current sensors need to be implemented
    AnalogIn V_Out_Positive = AnalogIn(PC_0);
    AnalogIn V_Out_Negative = AnalogIn(PC_1);
    DigitalOut nBMS_Fault_3V3 = DigitalOut(PC_5);
    DigitalIn SH_RESET_3V3 = DigitalIn(PA_2);
    // some configuration for this needs to be done at startup see mbedosce
    // CAN CAN_POWERTRAIN = CAN(PA_11, PA_12, 500000);
    SPI spiInterface = SPI(PB_5, PB_4, PA_5, PA_4, use_gpio_ssel);
    LTC681xParallelBus ltcBusInterface;
    CAN& CAN_POWERTRAIN; // reference to can object defined in main
};
