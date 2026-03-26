#include "BMSFaultDetection.h"

Timer t;

void chargingActions(BMS &BMSInstance){
		//current work in progress - needs to detect soc over can......
}	


void turnOffCellBalancing(BMS &BMSInstance){
	for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
		LTC6810::Configuration &config = BMSInstance.chips[i].getConfig();
		config.dischargeState = {.value = 0};
		BMSInstance.chips[i].updateConfig();
	}
}


void readCellVoltages(LTC681xParallelBus &ltcBusInterface, BMS &BMSInstance){

	//ltcBusInterface.WakeupBus();
	if(t.elapsed_time()>=100ms){
		BMSInstance.currentState=BMSInstance.FAULT;
		return;
	}

	bool tempsConverted = true;
	LTC681xParallelBus::LTC681xBusStatus stat = ltcBusInterface.WakeupBus();
	LTC681xParallelBus::BusCommand command = LTC681xParallelBus::BuildBroadcastBusCommand(StartCellVoltageADC(AdcMode::k7k, false, CellSelection::kAll));
	stat = ltcBusInterface.SendCommand(command);

	ThisThread::sleep_for(3ms);

	for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
		command = LTC681xParallelBus::BuildAddressedBusCommand(PollADCStatus(),i);
		stat = ltcBusInterface.PollAdcCompletion(command, 0);
		if(stat == LTC681xBus::LTC681xBusStatus::PollTimeout){
			// printf("ADC poll timeout, on Bank %d\n", i);
			tempsConverted = false;
			t.start();
			 // the rules require that we need to ensure we are getting data and that all sensors are working correctly, if we cannot get an adc conversion in 100ms this will thow a fault
		}
	}

	ThisThread::sleep_for(3ms);


	if(tempsConverted){
		t.stop();
		t.reset();
		//reset timer after successful adc conversions...
		for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
			uint8_t voltageReading[12] = {0};
			ltcBusInterface.WakeupBus();
			command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupA(), i);
			stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageReading);


			command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupB(), i);
			stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageReading+6);

			uint16_t* castVoltages = (uint16_t*)voltageReading;
			for(uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++){
			//volt = castVoltages[j];

				BMSInstance.voltages[i][j] = castVoltages[j];
			}
		//6 bytes per cell group reading (2 bytes per cell) ... transmitted in little endian 
		//casted so that its easier to read
		}
		BMSInstance.minVoltage = BMSInstance.voltages[0][0];
		BMSInstance.maxVoltage = BMSInstance.voltages[0][0];

		for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
			for(uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++){
				if(BMSInstance.voltages[i][j] < BMSInstance.minVoltage){
					BMSInstance.minVoltage = BMSInstance.voltages[i][j];
				}
				if(BMSInstance.voltages[i][j] > BMSInstance.maxVoltage){
					BMSInstance.maxVoltage = BMSInstance.voltages[i][j];
				}
			}
		}
	}


	for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
		uint8_t voltageReading[12] = {0};
		ltcBusInterface.WakeupBus();
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupA(), i);
		stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageReading);


		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupB(), i);
		stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageReading+6);

		uint16_t* castVoltages = (uint16_t*)voltageReading;
		for(uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++){
			//volt = castVoltages[j];

			BMSInstance.voltages[i][j] = castVoltages[j];
		}
		//6 bytes per cell group reading (2 bytes per cell) ... transmitted in little endian 
		//casted so that its easier to read

		

	}
	BMSInstance.minVoltage = BMSInstance.voltages[0][0];
	BMSInstance.maxVoltage = BMSInstance.voltages[0][0];

	for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
		for(uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++){
			if(BMSInstance.voltages[i][j] < BMSInstance.minVoltage){
				BMSInstance.minVoltage = BMSInstance.voltages[i][j];
			}
			if(BMSInstance.voltages[i][j] > BMSInstance.maxVoltage){
				BMSInstance.maxVoltage = BMSInstance.voltages[i][j];
			}
		}
	}


}


void readCellTemps(BMS &BMSInstance){
	for(uint8_t i  = 0; i < NUM_BATTERY_MODULES; i++){
		for(uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++){
			BMSInstance.cellTemps[i][j]= BMSInstance.chips[i].readTemperatureTMP1075(&BMSInstance.sensors[i][j]);
		}
	}
}




// balancing should be done while charging and most cells are most of the way charged, or when idle and most cells are mostly charged 
// our balancing threshold is at 85% of maximum charges
void decideBalancing(BMS &BMSInstance){
	//turns on balancing for chips 

	if(BMSInstance.maxVoltage >= BALANCING_THRESHOLD && BMSInstance.minVoltage > MIN_CELL_VOLTAGE){
		for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
			uint8_t dischargeValue = 0x00;
			LTC6810::Configuration &config = BMSInstance.chips[i].getConfig();
			// uint16_t moduleVolts[NUM_VOLTAGES_PER_MODULE] = BMSInstance.voltages[i];
			uint16_t minModuleVolt = BMSInstance.voltages[i][0];
			uint16_t maxModuleVolt = BMSInstance.voltages[i][0];
			for(uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++){
				if(BMSInstance.voltages[i][j] < minModuleVolt){
					minModuleVolt = BMSInstance.voltages[i][j];
				}
				if(BMSInstance.voltages[i][j] > maxModuleVolt){
					maxModuleVolt = BMSInstance.voltages[i][j];
				}
				if((maxModuleVolt - minModuleVolt) >= DIFFERENCE_THRESHOLD){
					dischargeValue |= (0x1<<j);
				}
			}
			config.dischargeState.value = dischargeValue;
			BMSInstance.chips[i].updateConfig();	
		}
	}
}



void throwFault(BMS &BMSInstance){	
	// uint8_t module_fault_index = 0; // battery module where fault occured 
	// uint8_t temp_index = 0; // temp sensor (within a module) where a fault was detected
	// uint8_t voltage_fault_index = 0; // voltage group (1 of 6 parrallel groups) where a fault was detected

	//remember to set the data in the BMSInstance


	for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
		// cell voltage based faults...
		for(uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++){
			uint16_t voltVal = BMSInstance.voltages[i][j];
			if(voltVal >= MAX_CELL_VOLTAGE || voltVal <= MIN_CELL_VOLTAGE){



				BMSInstance.currentState = BMSInstance.FAULT;

				BMSInstance.nBMS_Fault_3V3 = 0;
				// module_fault_index = i;
				// voltage_fault_index = j;

				//set status.....

				BMSInstance.bms_stat_message.bmsFault = true;
				if(voltVal >= MAX_CELL_VOLTAGE){
					BMSInstance.bms_stat_message.cell_too_high = true;
				}else if(voltVal <= MIN_CELL_VOLTAGE){
					BMSInstance.bms_stat_message.cell_too_low = true;
				}
				BMSInstance.bms_stat_message.fault_index = j;
				BMSInstance.bms_stat_message.module_fault_index = i;


			} 
		}

		//cell temp based faults...
		for(uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++){
			int8_t tempReading = BMSInstance.cellTemps[i][j];
			if(BMSInstance.currentState == BMSInstance.CHARGING){
				if(tempReading>=CHARGING_CELL_MAX){
					BMSInstance.nBMS_Fault_3V3 = 0;
					// module_fault_index = i;
					// temp_index = j;
					//NOT DONE HERE have to add some stuff for telemetry
				}else if(tempReading <= CHARGING_CELL_MIN){
					BMSInstance.nBMS_Fault_3V3 = 0;
					// module_fault_index = i;
					// temp_index = j;
				}
			}
		}
	}



	// do bus bar temperature based fault throwing here ..... 


}


void controller(LTC681xParallelBus &ltcBusInterface, BMS &BMSInstance){
	if(BMSInstance.currentState != BMSInstance.FAULT){
		chargingActions(BMSInstance);
		turnOffCellBalancing(BMSInstance);
		ThisThread::sleep_for(3ms);
		readCellVoltages(ltcBusInterface, BMSInstance);
		readCellTemps(BMSInstance);
		throwFault(BMSInstance);
		decideBalancing(BMSInstance);
		checkIMDStatus(BMSInstance);
	}else{
		turnOffCellBalancing(BMSInstance);
		//need to turn on indicator lights as well ..... 
	}

}
