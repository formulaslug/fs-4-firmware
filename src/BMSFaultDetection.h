#ifndef BMSFAULTDETECTION_H
#define BMSFAULTDETECTION_H

#include "mbed.h"
#include "BmsConfig.h"
#include "EnergusTempSensor.h"



/*
current WIP -for BMS fault detection
need to detect charging 
need to implement method to interface with sensors --- seems to be 12 per bank - learn addresses : based on how theyre wired
charging status - given thru can also not updated yet so idfk  


todo:
find specifications on the cell voltage limits when charging/discharging 
find the specifications on the cell temperature limits again when charging/discharging 
finish the bms status message
*/


void detectCharging();
void decideBalancing(LTC681xParallelBus&);
void turnOffCellBalancing();
void readCellVoltages(LTC681xParallelBus&);
void readCellTemps();
void decideBalancing();
void throwFault();
void generateStatusMessage();
void controller();
void setup();



/*
Okay so the dbc has not been updated for the charging board as of 3-2-2026 so the following is placeholder
we can write this based on dbc for fs3 but I don't know if thats valueable at this point
mainly here to remind us that it needs to be written 
*/




CANMessage msg;

void setup(){
	//assume all is well at startup, the bms should update this as 
	bms_stat_message.bmsFault = false;
	bms_stat_message.imdFault = false;
	bms_stat_message.shutdownState = false;
	bms_stat_message.prechargeDone = false;
	bms_stat_message.charging = false;
	bms_stat_message.isBalancing = false;
	bms_stat_message.cell_too_low = false;
	bms_stat_message.cell_too_high = false;
	bms_stat_message.temp_too_low = false;
	bms_stat_message.temp_too_high = false;
	bms_stat_message.temp_too_high_charging = false;
	bms_stat_message.cell_fault_index = -1; // rememeber these are unsigned so 
	bms_stat_message.bank_fault_index = -1;


}

void detectCharging(){
	// This is essentially fs3 code - need to make sure this works 
	// i dont like how this is setup will make a new version later 
	canInterface->read(msg);
	uint32_t messageId = msg.id;
	uint8_t* data = msg.data;
	if(currentState==charging&&currentState!=fault&&currentState!=anomoly){
		if (messageId == 0x77){
			currentState = idle;
		}
		if(messageId == 0x78){ 
			currentState = charging;	
		}
	}
}	


void turnOffCellBalancing(){
	for(uint8_t i = 0; i < BATTERYBANKS; i++){
		LTC6810::Configuration &config = chips[i].getConfig();
		config.dischargeState = {.value = 0};
		chips[i].updateConfig();
	}


}


void readCellVoltages(LTC681xParallelBus &ltcBusInterface){

	//ltcBusInterface.WakeupBus();

	LTC681xParallelBus::LTC681xBusStatus stat = ltcBusInterface.WakeupBus();
	LTC681xParallelBus::BusCommand command = LTC681xParallelBus::BuildBroadcastBusCommand(StartCellVoltageADC(AdcMode::k7k, false, CellSelection::kAll));
	stat = ltcBusInterface.SendCommand(command);

	ThisThread::sleep_for(3ms);

	for(uint8_t i = 0; i < BATTERYBANKS; i++){
		command = LTC681xParallelBus::BuildAddressedBusCommand(PollADCStatus(),i);
		stat = ltcBusInterface.PollAdcCompletion(command, 0);
		if(stat == LTC681xBus::LTC681xBusStatus::PollTimeout){
			printf("ADC poll timeout, on Bank %d\n", i);
		}
	}

	ThisThread::sleep_for(3ms);

	//status checks should be added here .... 

	for(uint8_t i = 0; i < BATTERYBANKS; i++){
		uint8_t voltageReading[12] = {0};
		ltcBusInterface.WakeupBus();
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupA(), i);
		stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageReading);


		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupB(), i);
		stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageReading+6);

		uint16_t* castVoltages = (uint16_t*)voltageReading;
		for(uint8_t j = 0; j < NUMCELLSPERBANK; j++){
			//volt = castVoltages[j];

			voltages[BATTERYBANKS*i+j] = castVoltages[j];
			//might be a better way to do this but for now...
		}
		//6 bytes per cell group reading (2 bytes per cell) ... transmitted in little endian 
		//casted so that its easier to read

		

	}
	minVoltage = voltages[0];
	maxVoltage = voltages[0];
	for(uint8_t i = 0; i < BATTERYBANKS*NUMCELLSPERBANK; i++){
		if(voltages[i] < minVoltage){
			minVoltage = voltages[i];
		}
		if(voltages[i] > maxVoltage){
			maxVoltage = voltages[i];
		}
	}

}


void readCellTemps(){
	for (int i = 0; i < BATTERYBANKS; i++) {
       	chips[i].getConfig().gpio4 = LTC6810::GPIOOutputState::kHigh;
        chips[i].getConfig().gpio3 = LTC6810::GPIOOutputState::kHigh;
    }

	for(uint8_t i = 0; i < BATTERYBANKS; i++){
		cellTemps[i] = chips[i].readTemperatureTMP1075(&sensors[i]);			
	}
}




// balancing should be done while charging and most cells are most of the way charged, or when idle and most cells are mostly charged 
void decideBalancing(){
	//turns on balancing for chips 
	for(uint8_t i = 0; i < BATTERYBANKS; i++){
		uint8_t dischargeValue = 0x00;
		LTC6810::Configuration &config = chips[i].getConfig();
		for(uint8_t j = 0; j < NUMCELLSPERBANK; j++){
			uint16_t volts = voltages[i*BATTERYBANKS+j];
			if(volts >= BALANCETHRESHOLD && volts >= minVoltage + BALANCETHRESHOLD){ // need to find the min voltage
				dischargeValue |= (0x1<<j);
			}
		}
		config.dischargeState.value = dischargeValue;
		chips[i].updateConfig();
	}

}



void throwFault(){
	uint8_t voltageBankFault = 0;
	uint8_t tempBankFault = 0;
	uint8_t cellFault = 0; 
	for(uint8_t i = 0; i < BATTERYBANKS*NUMCELLSPERBANK; i++){
		uint16_t voltVal = voltages[i];
		if(voltVal >= BMSMAXVOLT || voltVal <= BMSMINVOLT ){
			if(currentState == idle || currentState == charging){
				currentState = anomoly;
			}else if(currentState == anomoly){
				currentState = fault;
				voltageBankFault = i/5;
				cellFault = i%(voltageBankFault*5);
				//open the shutdown circuit, assuming that its a low to open it.
				shutdownPin = 0;



				if (voltVal <= BMSMINVOLT){
					bms_stat_message.cell_too_low = true;
				}else if(voltVal >= BMSMAXVOLT){
					bms_stat_message.cell_too_high = true;
				}
				bms_stat_message.cell_fault_index = cellFault;
				bms_stat_message.bank_fault_index = voltageBankFault;


			}	
		}
	}
	// need to update fault throwing when charging vs when not charging 
	// im going to leave this as is until we have a better idea of how the tmp1075s are setup 
	for(uint8_t i = 0; i < BATTERYBANKS; i++){
		float temperature = cellTemps[i];
		if(temperature >= MAXCELLTEMP || temperature <= MINCELLTEMP){
			if(currentState == idle || currentState == charging){
				currentState = anomoly;
			}else if(currentState == anomoly){
				currentState = fault;
				tempBankFault = i;
			}
		}
	}

}


void controller(LTC681xParallelBus &ltcBusInterface){
	if(currentState != fault){
		turnOffCellBalancing();
		ThisThread::sleep_for(3ms);
		readCellVoltages(ltcBusInterface);
		readCellTemps();
		throwFault();
	}
	if(currentState == charging){
		decideBalancing();
	}
}


#endif
