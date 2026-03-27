#ifndef BMSFAULTDETECTION_H

#define BMSFAULTDETECTION_H

#include "mbed.h"
#include "BmsConfig.h"


void chargingActions();
void decideBalancing(BMS&);
void turnOffCellBalancing(BMS&);
void readCellVoltages(LTC681xParallelBus&, BMS&);
void readCellTemps(BMS&);
void decideBalancing(BMS&);
void checkForFaults(BMS&);
void generateStatusMessage(BMS&);
void controller(LTC681xParallelBus&, BMS&);
void checkShutdownCircuit(BMS&);


#endif
