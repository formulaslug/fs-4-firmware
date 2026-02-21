#include "mbed.h"

#ifndef BATTERYBANKS
#define BATTERYBANKS 5
#endif

#ifndef SPIMISO
#define SPIMISO PA_20
#endif

#ifndef SPIMOSI
#define SPIMOSI PA_28
#endif

#ifndef BMSCS
#define BMSCS PA_12
#endif

#ifndef CLK
#define CLK PA_15
#endif

#ifndef NRST
#define NRST PG_10
#endif

#ifndef PRECHARGECTRL
#define PRECHARGECTRL PB_4
#endif

#ifndef FANPWM
#define FANPWM PB_6
#endif

#ifndef GLVMEASURE
#define GLVMEASURE PB_7
#endif

#ifndef CANTD
#define CANTD PA_0
#endif

#ifndef CANRD
#define CANRD PA_1
#endif

#ifndef BMSFAULTIND
#define BMSFAULTIND PA_5
#endif

#ifndef IMDFAULT
#define IMDFAULT PA_6
#endif

#ifndef EMETERPARASITE
#define EMETERPARASITE PA_4
#endif

#ifndef EMETERMOUNT
#define EMETERMOUNT PA_7
#endif

#ifndef SHUTDOWNMEASURE
#define SHUTDOWNMEASURE PA_9
#endif

#ifndef CURRENTSENSE
#define CURRENTSENSE PA_11
#endif

#ifndef TXVCPUART
#define TXVCPUART PA_2
#endif

#ifndef RXVCPUART
#define RXVCPUART PA_3
#endif

#ifndef OSC_IN
#define OSC_IN PF_0
#endif

#ifndef OSC_OUT
#define OSC_OUT PF_1
#endif


#ifndef MAXCELLVOLTAGE
#define MAXCELLVOLTAGE
#endif


#ifndef PLACEHOLDERCELLS
#define PLACEHOLDERCELLS 5
#endif

#ifndef NUMCELLSPERBANK
#define NUMCELLSPERBANK 5
#endif


#ifndef BALANCETHRESHOLD
#define BALANCETHRESHOLD 3900
#endif

#ifndef DISCHARGE_THRESHOLD
#define DISCHARGE_THRESHOLD 5
#endif

//bms max and min volt are just placeholders currently, need to read the data sheets of the cells being used
#ifndef BMSMAXVOLT
#define BMSMAXVOLT 4000
#endif

#ifndef BMSMINVOLT
#define BMSMINVOLT 2000
#endif


#ifndef MAXCELLTEMP
#define MAXCELLTEMP 100.0
#endif


#ifndef MINCELLTEMP
#define MINCELLTEMP 0.0
#endif

struct TMP1075_Handle_t{
  uint8_t i2c_address;
  uint8_t temp_reg;
};

std::vector<LTC6810> chips;
// std::vector<TMP1075_Handle_t> sensors;
LTC6810::TMP1075_Handle_t sensors[BATTERYBANKS];
//uint16_t voltages[BATTERYBANKS*NUMCELLSPERBANK];



struct status_msg {
  bool bmsFault;
  bool imdFault;
  bool shutdownState;
  bool prechargeDone;
  bool precharging;
  bool charging;
  bool isBalancing;
  bool cell_too_low;
  bool cell_too_high;
  bool temp_too_low;
  bool temp_too_high;
  bool temp_too_high_charging;
  uint16_t glv_voltage;
  uint32_t cell_fault_index;
};

struct tray_temps_msg {
    uint8_t temp_busbar;
    uint8_t temp_pack_fuse;
    uint8_t temp_bolted_connection;
};


uint16_t voltages[BATTERYBANKS*NUMCELLSPERBANK];
float cellTemps[BATTERYBANKS];
uint16_t minVoltage;
uint16_t maxVoltage;



enum bms_state{
  idle,
  charging,
  anomoly,
  fault  
};
bms_state state;
status_msg CurrentBMSStatus;




