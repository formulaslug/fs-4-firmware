#ifndef BMSFAULTDETECTION_H

#define BMSFAULTDETECTION_H

#include "mbed.h"
#include "BmsConfig.h"


void chargingActions();
void decideBalancing(BMS&);
void turnOffCellBalancing(BMS&);
void readCellVoltages(LTC681xParallelBus&, BMS&);
void readTemps(BMS&);
void decideBalancing(BMS&);
void checkForFaults(BMS&);
void generateStatusMessage(BMS&);
void controlFans(BMS &BMSInstance);
void controller(LTC681xParallelBus&, BMS&);
void checkShutdownCircuit(BMS&);
void checkIMDStatus(BMS&);

//for the current sensor
void calibrateCurrentSensor(BMS&);
void readPackCurrent(BMS&);
float getPackCurrentAmps(BMS&);



#endif
