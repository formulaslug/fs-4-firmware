#include "BMSFaultDetection.h"


// need to add some sort of logging for testing - over uart



/*
	move structs and telemetry data to seperate can class - call can class every so often using data from BMS instance 
	move BMSfaultDetection functions to BMS class so we dont have to keep passing a reference (in the event queue we would make controller a public function and call that )
	make changes to precharge ... soon - note ideally precharge runs once per startup we do NOT close the precharge relay while the car is running
	(add real tmp1075 addresses)




	note for precharge - precharge should be run once during startup and once after each time the shutdown circuit is open 

	- implementing current sensor is underway make sure changes are announced beforehand





*/





void chargingActions(BMS &BMSInstance){
		//current work in progress - needs to detect soc over can......
	//code from fs3 adapted to fs4
		uint8_t canData = 0;
        BMSInstance.CAN_POWERTRAIN.read(BMSInstance.msg);
            uint32_t id = BMSInstance.msg.id;
            unsigned char* data = BMSInstance.msg.data;

            if (!(BMSInstance.currentState==BMSInstance.FAULT)){
                switch (id) {
                case 0x682: // temperature message from MC
                    canData = (data[2] | (data[3] << 8));
                    break;
                default:
                    break;
                }
            } else {
                switch (id) {
                case 0x190: // charge status from charger, 180 + node ID (10)
                    canData = (data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)) / 100;
                default:
                    break;
                }
            }
    // we still need to do safety checks on this --- perhaps we can store the data from this in BMSInstance and then have checks in checkForFaults

}	


void turnOffCellBalancing(BMS &BMSInstance){
	for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
		LTC6810::Configuration &config = BMSInstance.chips[i].getConfig();
		config.dischargeState = {.value = 0};
		BMSInstance.chips[i].updateConfig();
	}
	printf("Cell balancing deactivated....\n");
	// char msg2[] = "cell balancing deactivated\n";
	// BMSInstance.VCP_UART.write(msg2, sizeof(msg2));
}


void readCellVoltages(LTC681xParallelBus &ltcBusInterface, BMS &BMSInstance){
	// char msg3[] = "Reading Cell Voltages\n";
	// BMSInstance.VCP_UART.write(msg3, sizeof(msg3));
	printf("Reading cell voltages...\n");
	//ltcBusInterface.WakeupBus();
	if(BMSInstance.ltcTimeoutTimer.elapsed_time()>=100ms){
		BMSInstance.currentState=BMSInstance.FAULT;
		return;
	}

	bool voltsConverted = true;
	LTC681xParallelBus::LTC681xBusStatus stat = ltcBusInterface.WakeupBus();
	LTC681xParallelBus::BusCommand command = LTC681xParallelBus::BuildBroadcastBusCommand(StartCellVoltageADC(AdcMode::k7k, false, CellSelection::kAll));
	stat = ltcBusInterface.SendCommand(command);

	ThisThread::sleep_for(3ms);

	for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
		command = LTC681xParallelBus::BuildAddressedBusCommand(PollADCStatus(),i);
		stat = ltcBusInterface.PollAdcCompletion(command, 0);

		if(stat == LTC681xBus::LTC681xBusStatus::PollTimeout){
			// printf("ADC poll timeout, on Bank %d\n", i);
			voltsConverted = false;
			BMSInstance.ltcTimeoutTimer.start();
			printf("poll timeout occured...\n");
			// char msg4[] = "poll timeout occured\n";
			// BMSInstance.VCP_UART.write(msg4, sizeof(msg4));
			// the rules require that we need to ensure we are getting data and that all sensors are working correctly, if we cannot get an adc conversion in 100ms this will thow a fault
			// that time period is a little arbitrary and probably should be adjusted 
		}
	}

	ThisThread::sleep_for(3ms);


	if(voltsConverted){
		BMSInstance.ltcTimeoutTimer.stop();
		BMSInstance.ltcTimeoutTimer.reset();
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
				printf("%d\n", castVoltages[j]); // printing the cell voltages for testing purposes

				BMSInstance.voltages[i][j] = castVoltages[j];
			}
			printf("\n");
		//6 bytes per cell group reading (2 bytes per cell) ... transmitted in little endian 
		//casted so that its easier to read
		}
		// char msg5[] = "successfully read voltages\n";
		// BMSInstance.VCP_UART.write(msg5, sizeof(msg5));
		printf("read voltages...\n");
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


}


void readTemps(BMS &BMSInstance){
	// char msg6[] = "reading cell temps\n";
	// BMSInstance.VCP_UART.write(msg6, sizeof(msg6));
	printf("reading cell temps\n");

	for(uint8_t i  = 0; i < NUM_BATTERY_MODULES; i++){
		for(uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++){
			BMSInstance.cellTemps[i][j]= BMSInstance.chips[i].readTemperatureTMP1075(&BMSInstance.sensors[i][j]);
		}
	}

	int8_t maxTemp = 0;

    // Find the hottest cell across all modules and sensors
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            if (BMSInstance.cellTemps[i][j] > maxTemp) {
                maxTemp = BMSInstance.cellTemps[i][j];
            }
        }
    }
    BMSInstance.maxCellTemp = maxTemp;


    for(uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS; i++){
		BMSInstance.trayTempSensors[i].start_conversion(true); // assume no e meter here CHANGE LATER
		ThisThread::sleep_for(3ms);
		uint8_t trayTemp = BMSInstance.trayTempSensors[i].retrieve_conversion();
		BMSInstance.trayTemps[i] = trayTemp;
	}

}




// balancing should be done while charging and most cells are most of the way charged, or when idle and most cells are mostly charged 
// our balancing threshold is at 85% of maximum charges
void decideBalancing(BMS &BMSInstance){
	//turns on balancing for chips 
	// char msg7[] = "deciding balancing\n";
	// BMSInstance.VCP_UART.write(msg7, sizeof(msg7));
	printf("deciding balancing......");
	if(BMSInstance.currentState!=BMSInstance.FAULT){
		// do we balance based on the whole battery or per module? - need to ask
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
}

void checkIMDStatus(BMS &BMSInstance){



}



void checkForFaults(BMS &BMSInstance){	
	// uint8_t module_fault_index = 0; // battery module where fault occured 
	// uint8_t temp_index = 0; // temp sensor (within a module) where a fault was detected
	// uint8_t voltage_fault_index = 0; // voltage group (1 of 6 parrallel groups) where a fault was detected

	//remember to set the data in the BMSInstance

	// char msg8[] = "reading cell temps\n";
	// BMSInstance.VCP_UART.write(msg8, sizeof(msg8));
	printf("reading cell temps ");
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
					BMSInstance.currentState = BMSInstance.FAULT;
					BMSInstance.nBMS_Fault_3V3 = 0;
					// module_fault_index = i;
					// temp_index = j;
					//NOT DONE HERE have to add some stuff for telemetry
				}else if(tempReading <= CHARGING_CELL_MIN){
					BMSInstance.currentState = BMSInstance.FAULT;
					BMSInstance.nBMS_Fault_3V3 = 0;
					// module_fault_index = i;
					// temp_index = j;
				}
			}
		}
	}

	//tray temp sensor checks
	// for testing purposes i am going to use the cell temperature limits here, will be updated later
	//telemetry stuff needs to be added but the logic is there. 
	//
	// char msg9[] = "reading tray temp sensors\n";
	// BMSInstance.VCP_UART.write(msg9, sizeof(msg9));
	for(uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS; i++){
		uint8_t trayTemp = BMSInstance.trayTemps[i];
		if(trayTemp >= CELL_MAX || trayTemp <= CELL_MIN){
			//should not cause a fault just for data 4-4-26
			BMSInstance.currentState = BMSInstance.FAULT;
			BMSInstance.nBMS_Fault_3V3 = 0;
		}
	}
	


}


// i am undecided wether or not to put the bms into a fault state here - might add a state that doesnt turn on any bms indicator lights but essentially stops the bms functions like in a fault state
// not totally sure whats required of the bms in this case making a best guess
/*
	only one we care about is the final shutdown circuit reading - we only care about it for the purposes of precharing again after a shutdown

	the other shutdown circuit inputs should be reported to can 4-4-26


*/



void checkShutdownCircuit(BMS &BMSInstance){
	if(BMSInstance.Shutdown_In_3V3_Filtered.read()==0 || BMSInstance.Shutdown_Out_3V3_Filtered.read()==0){
		// shutdown circuit open before the bms
		turnOffCellBalancing(BMSInstance);
		//precharge should also be turned off but thats in a different task 
		if(BMSInstance.Shutdown_Out_3V3_Filtered.read()==0){
			//shutdown circuit opened after the bms
			turnOffCellBalancing(BMSInstance);
		}
		if(BMSInstance.Shutdown_In_3V3_Filtered.read()==0){
			turnOffCellBalancing(BMSInstance);
		}
	}

}


void controlFans(BMS &BMSInstance) {
    int8_t maxTemp = BMSInstance.maxCellTemp;

    if (BMSInstance.currentState != BMSInstance.PRECHARGING) {
        // Linear scaling: 20% at ~20°C, 100% at ~50°C
        // Formula: (2.6667 * temp) - 33.3333, clamped to [20, 100]
        int raw_percent = (int)((2.6667f * maxTemp) - 33.3333f);
        uint8_t fan_percent = (uint8_t)std::clamp(raw_percent, 20, 100);
        BMSInstance.Fan_PWM.write(fan_percent / 100.0f); // PWM expects 0.0 - 1.0
    } else {
        // Keep fans off until precharge is complete
        BMSInstance.Fan_PWM.write(0.0f);
    }
}

void controller(LTC681xParallelBus &ltcBusInterface, BMS &BMSInstance){
	printf("controller functions...\n");
	if(BMSInstance.currentState != BMSInstance.FAULT){
		//chargingActions(BMSInstance);
		turnOffCellBalancing(BMSInstance);
		ThisThread::sleep_for(3ms);
		readCellVoltages(ltcBusInterface, BMSInstance);
		readTemps(BMSInstance);
		checkForFaults(BMSInstance);
		controlFans(BMSInstance);
		decideBalancing(BMSInstance);
		//checkShutdownCircuit(BMSInstance);
	}else{
		turnOffCellBalancing(BMSInstance);
		//need to turn on indicator lights as well ..... 
		// precharge relay should be open in this case
		//fans turn off on fault
		 BMSInstance.Fan_PWM.write(0.0f);

	}

}