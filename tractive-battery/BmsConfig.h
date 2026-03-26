#include "mbed.h"
#include "LTC6810.h"
#include "LTC681xBus.h"
#include "LTC681xParallelBus.h"
#include "LTC681xCommand.h"
#include "DS18B20.h"

// #include ""
//general config
inline constexpr uint8_t NUM_BATTERY_MODULES = 5; 
inline constexpr uint8_t NUM_VOLTAGES_PER_MODULE = 6;
inline constexpr uint8_t NUM_TEMP_SENSORS_PER_MODULE = 12;
//okay so ive looked at the kicad and these have not been wired differently so 11/12 sensors have the address of 0x48 (based on the data sheet) based on how they are wired
// i assume this is a wip and will be updated later
inline constexpr uint16_t TMP1075_ADDRESSES[NUM_TEMP_SENSORS_PER_MODULE] = {0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48,0x48, 0x48, 0x48, 0x48}; 



//battery cell info for inr-18650-p30b

inline constexpr int8_t CHARGING_CELL_MAX = 60;
inline constexpr int8_t CHARGING_CELL_MIN = 0;
inline constexpr int8_t CELL_MAX = 60;
inline constexpr int8_t CELL_MIN = -40;

inline constexpr uint16_t MAX_CELL_VOLTAGE = 42000; // 4.2 volts
inline constexpr uint16_t MIN_CELL_VOLTAGE = 25000; // 2.5 volts

inline constexpr uint16_t BALANCING_THRESHOLD = 35700;// 3.57 volts
inline constexpr uint16_t DIFFERENCE_THRESHOLD = 00300; // 30 milivolts





class BMS{
  public:
    struct TMP1075_Handle_t{
      uint8_t i2c_address;
      uint8_t temp_reg;
    };


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


    struct powerPerformanceData{
      uint16_t packVoltage;
      uint16_t packCurrent;
      uint8_t Soc;
      uint8_t fanPWM;
      uint32_t instantPwr;
    };


    struct ThermalStats{ 
    // not totally sure how to implement this it involves sensor data the BMS doesnt have access to
    // given we have a seperate cell temperature message maybe we dont have to? ill keep this here for now
      int8_t maxCellTemp;
      int8_t minCellTemp;
      int8_t avgCellTemp;
      int8_t boltedConnectionTemp;
      int8_t busBarTemp;
      int8_t packFuseTemp;
      int8_t intake_air_temp;
      int8_t cowling_exhaustTemp;
    };

    struct cellVoltages{
      uint16_t module1Volts;
      uint16_t module2Volts;
      uint16_t module3Volts;
      uint16_t module4Volts;
      uint16_t module5Volts;
    };

    struct cellTemperatures{
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

    int8_t cellTemps[NUM_BATTERY_MODULES][NUM_TEMP_SENSORS_PER_MODULE];
    uint16_t minVoltage;
    uint16_t maxVoltage;

    std::vector<LTC6810> chips;
    LTC6810::TMP1075_Handle_t sensors[NUM_BATTERY_MODULES][NUM_TEMP_SENSORS_PER_MODULE];
    uint16_t voltages[NUM_BATTERY_MODULES][NUM_VOLTAGES_PER_MODULE];
    enum bms_state{
      ACTIVE = 0,
      CHARGING = 1,
      FAULT = 2  
    };

      //pin definitions 
      AnalogIn V_Out_Positive = AnalogIn(PC_0);
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
      SPI spiInterface = SPI(PB_4, PB_5, PB_10, PB_9, use_gpio_ssel);
      DigitalOut TS1W_PU_Control = DigitalOut(PB_15);
      OneWire TS1W = OneWire(PB_14); // look up more on 1 wire interface 



      // temperature sensor interface
      // dont know ids but will be implemented here 
      //


    bms_state currentState;
    status_msg bms_stat_message;

    BMS();

};
