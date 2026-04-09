#include "mbed.h"
#include "LTC6810.h"
#include "LTC681xBus.h"
#include "LTC681xParallelBus.h"
#include "LTC681xCommand.h"
#include "DS18B20.h"

//general config
inline constexpr uint8_t NUM_BATTERY_MODULES = 5; 
inline constexpr uint8_t NUM_VOLTAGES_PER_MODULE = 6;
inline constexpr uint8_t NUM_TEMP_SENSORS_PER_MODULE = 12;
inline constexpr uint8_t NUM_TRAY_TEMP_SENSORS = 4;
//okay so ive looked at the kicad and these have not been wired differently so 11/12 sensors have the address of 0x48 (based on the data sheet) based on how they are wired
// i assume this is a wip and will be updated later
inline constexpr uint16_t TMP1075_ADDRESSES[NUM_TEMP_SENSORS_PER_MODULE] = {0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48,0x48, 0x48, 0x48, 0x48}; 

//this is a temporary thing - i dont know the actual addresses - 64 bit unique address for each sensor 
inline constexpr uint64_t TRAYTEMP_SENSOR_ADDRESSES[NUM_TRAY_TEMP_SENSORS] = {0x00000000,0x00000000,0x00000000,0x00000000};


//battery cell info for inr-18650-p30b - based on datasheet
inline constexpr int8_t CHARGING_CELL_MAX = 60;
inline constexpr int8_t CHARGING_CELL_MIN = 0;
inline constexpr int8_t CELL_MAX = 60;
inline constexpr int8_t CELL_MIN = -40;

inline constexpr uint16_t MAX_CELL_VOLTAGE = 42000; // 4.2 volts
inline constexpr uint16_t MIN_CELL_VOLTAGE = 25000; // 2.5 volts

inline constexpr uint16_t BALANCING_THRESHOLD = 35700;// 3.57 volts
inline constexpr uint16_t DIFFERENCE_THRESHOLD = 00300; // 30 milivolts

// HASS 300-S current sensor constants (from datasheet)
inline constexpr float HASS300_IPN            = 300.0f;                      // Nominal primary current (A)
inline constexpr float HASS300_SENSITIVITY    = 0.625f / HASS300_IPN;        // V/A = ~0.002083 V/A
inline constexpr float HASS300_VREF           = 2.5f;                        // Output voltage at zero current (V)
inline constexpr float HASS300_ADC_REF        = 3.3f;                        // MCU ADC reference voltage (V)
inline constexpr float MAX_PACK_CURRENT_AMPS  = 1000.0f;                     // Overcurrent fault threshold - tune later
inline constexpr size_t CURRENT_FILTER_SAMPLES = 10;                         // Rolling average window size




class BMS{
  private:

    void chargingActions();
    void decideBalancing();
    void turnOffCellBalancing();
    void readCellVoltages();
    void readTemps();
    void checkForFaults();
    void generateStatusMessage();
    void controlFans();
    void checkShutdownCircuit();
    void checkIMDStatus();
    void readPackCurrent();

    enum bms_state{
        ACTIVE = 0,
        CHARGING = 1,
        FAULT = 2,
        PRECHARGING = 3
    };

    bms_state currentState;

  public:
    BMS();
    void controller();

    struct TMP1075_Handle_t{
      uint8_t i2c_address;
      uint8_t temp_reg;
    };

    Timer ltcTimeoutTimer;
    CANMessage msg;

    int8_t cellTemps[NUM_BATTERY_MODULES][NUM_TEMP_SENSORS_PER_MODULE];
    uint16_t minVoltage;
    uint16_t maxVoltage;
    uint16_t maxCellTemp;

    std::vector<LTC6810> chips;
    LTC6810::TMP1075_Handle_t sensors[NUM_BATTERY_MODULES][NUM_TEMP_SENSORS_PER_MODULE];
    uint16_t voltages[NUM_BATTERY_MODULES][NUM_VOLTAGES_PER_MODULE];

    std::vector<DS18B20> trayTempSensors;

    uint8_t trayTemps[NUM_TRAY_TEMP_SENSORS];
      //pin definitions 
      AnalogIn V_Out_Positive = AnalogIn(PC_0); // current sensors need to be implemented 
      AnalogIn V_Out_Negative = AnalogIn(PC_1);
      DigitalIn Charge_State_Filtered = DigitalIn(PC_2); // assume 1 for charging 0 for not charging
      DigitalIn IMD_Fault_3V3 = DigitalIn(PC_4);
      DigitalOut nBMS_Fault_3V3 = DigitalOut(PC_5);
      PwmOut Fan_PWM = PwmOut(PC_8);
      DigitalOut TS_READY = DigitalOut(PC_9); //look more into this one (i think its a precharge indicator )
      DigitalIn Shutdown_In_3V3_Filtered = DigitalIn(PA_0); // status of the shutdown circuit before bms
      DigitalIn Shutdown_Out_3V3_Filtered = DigitalIn(PA_1); // status of the shutdown circuit after bms
      DigitalIn SH_RESET_3V3 = DigitalIn(PA_2); 
      DigitalIn Shutdown_Measure = DigitalIn(PA_6);
      AnalogIn GLV_Voltage = AnalogIn(PA_7);
      BufferedSerial VCP_UART = BufferedSerial(PA_9, PA_10); // some configuration for this needs to be done at startup see mbedosce
      CAN CAN_POWERTRAIN = CAN(PA_11, PA_12);
      DigitalOut nPrechargeControl = DigitalOut(PB_0);
      SPI spiInterface = SPI(PB_5, PB_4, PB_10, PB_9, use_gpio_ssel);
      LTC681xParallelBus ltcBusInterface;
      DigitalOut TS1W_PU_Control = DigitalOut(PB_15);
      OneWire TS1W = OneWire(PB_14); // look up more on 1 wire interface 
      // temperature sensor interface
      // dont know ids but will be implemented here 
      float packCurrentAmps;
};
