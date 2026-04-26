#ifndef BMSFAULTDETECTION_H

#define BMSFAULTDETECTION_H

#include "BmsConfig.h"
#include "mbed.h"

void chargingActions();
void decideBalancing(BMS&);
void turnOffCellBalancing(BMS&);
void readCellVoltages(LTC681xParallelBus&, BMS&);
void readTemps(BMS&);
void decideBalancing(BMS&);
void readPackCurrent(BMS& BMSInstance);
void checkForFaults(BMS&);
void generateStatusMessage(BMS&);
void controlFans(BMS& BMSInstance);
void controller(LTC681xParallelBus&, BMS&);
void checkShutdownCircuit(BMS&);
void checkIMDStatus(BMS&);

#endif
