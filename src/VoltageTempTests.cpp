#define CS PA_4
#define MOSI PB_5
#define MISO PB_4
#define CLK PB_3
#define NUM_CELL_BANKS 5
#define NUM_CELLS_PER_BANK 6



#include "mbed.h"
#include "LTC681xBus.h"
#include "LTC681xParallelBus.h"
#include "LTC6811.h"
#include "ThisThread.h"
#include "EnergusTempSensor.h"
#include <vector>


EventQueue queue(3*EVENTS_EVENT_SIZE);


SPI* spiInterface;
std::vector<LTC6811> chips;
const int BMS_CELL_MAP[12] = {0, 1, 2, -1, -1, -1, 3, 4, 5, -1, -1, -1};





void initalizeIO();
void selfChecks(LTC681xParallelBus&);
void checkCellVoltages(LTC681xParallelBus&);
void checkTemperatures(LTC681xParallelBus&);
void turnOffCellBalancing();



void initalizeIO(){
	spiInterface = new SPI(MOSI, MISO, CLK, CS);
	spiInterface->format(8,0);
	//init can and fans here later 
}



void turnOffCellBalancing(){
	for(uint8_t i = 0; i < NUM_CELL_BANKS; i++){
		LTC6811::Configuration &config = chips[i].getConfig();
		config.dischargeState.value = 0x0000;
		chips[i].updateConfig();
	}


}



void selfChecks(LTC681xParallelBus& ltcBusInterface){
	ltcBusInterface.WakeupBus();
	LTC681xBus::BusCommand command = LTC681xBus::BuildBroadcastBusCommand(StartSelfTestCellVoltage(AdcMode::k7k, SelfTestMode::kSelfTest1));
	ltcBusInterface.SendCommand(command);
	uint8_t results[24] = {0};

	ThisThread::sleep_for(3ms);
	//wait for the command to finish and all ltc chips to go back to a different state
	ltcBusInterface.WakeupBus();
	for(uint8_t i = 0; i < NUM_CELL_BANKS; i++){
		// ltcBusInterface.WakeupBus();
		//read cells 0-2
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupA(), i);
		ltcBusInterface.SendReadCommand(command, (uint8_t*)results);
		//read cells 3-5
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupB(), i);
		ltcBusInterface.SendReadCommand(command, (uint8_t*)results+6);
		//read cells 6-8
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupC(), i);
		ltcBusInterface.SendReadCommand(command,(uint8_t*)results+12);
		//read cells 9-11
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupD(), i);
		ltcBusInterface.SendReadCommand(command, (uint8_t*)results+18);

		// it sends the data as little endian - converting from uint8_t to uint16_t makes it much easier to work with

		uint16_t *realResults = (uint16_t*)results;
		printf("results should be alternating 0s and 1s....\n");
		printf("results for bank %d\n", i);
		for(uint8_t j = 0; j < 12; j++){
			printf("%x\n", realResults[j]);
		}

	}

	printf("testing gpio adc.....\n\n");

	uint8_t GpioTestResults[12] = {0};
	ThisThread::sleep_for(3ms);

	command = LTC681xBus::BuildBroadcastBusCommand(StartSelfTestGpio(AdcMode::k7k, SelfTestMode::kSelfTest1));
	ltcBusInterface.WakeupBus();

	ltcBusInterface.SendCommand(command);

	for(uint8_t i = 0; i < NUM_CELL_BANKS; i++){
		//ltcBusInterface.WakeupBus();
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupA(), i);
		ltcBusInterface.SendReadCommand(command, (uint8_t*)GpioTestResults);
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupB(),i);
		ltcBusInterface.SendReadCommand(command, (uint8_t*)GpioTestResults+6);

		printf("Results for gpio adc %d: should be alternating 0s and 1s...\n", i);
		uint16_t* RealGpioTest = (uint16_t*)GpioTestResults;
		for(uint8_t j = 0; j < 6; j++){
			printf("%x\n", RealGpioTest[j]);
		}
	}
}


void checkCellVoltages(LTC681xParallelBus& ltcBusInterface){

	printf("check cell voltage \n");
	
	turnOffCellBalancing();

	LTC681xBus::BusCommand command = LTC681xParallelBus::BuildBroadcastBusCommand(StartCellVoltageADC(AdcMode::k7k, false, CellSelection::kAll));
	ltcBusInterface.WakeupBus();
	// stat = ltcBusInterface.SendCommand(command);

	LTC681xBus::LTC681xBusStatus stat = ltcBusInterface.SendCommand(command);
	

	if(stat==LTC681xBus::LTC681xBusStatus::Ok){
		printf("Sent Command To start cell ADC");

	}else{
		printf("could not send start command to ADC... returning....\n");
		return;
	}


	ThisThread::sleep_for(3ms);
	ltcBusInterface.WakeupBus();
	for(uint8_t i = 0; i < NUM_CELL_BANKS; i++){
		command = LTC681xParallelBus::BuildAddressedBusCommand(PollADCStatus(), i);
		stat = ltcBusInterface.SendCommand(command);

		if(stat == LTC681xBus::LTC681xBusStatus::PollTimeout){
			printf("Reached ADC poll timeout on bank %d ... stopping voltage read\n", i);
			return;
		}
	}
	ThisThread::sleep_for(3ms);
	uint8_t voltageRawResults[24] = {};
	ltcBusInterface.WakeupBus();
	for(uint8_t i = 0; i < NUM_CELL_BANKS; i++){
		//ltcBusInterface.WakeupBus();
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupA(), i);
		stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageRawResults);
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupB(), i);
		stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageRawResults+6);
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupC(), i);
		stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageRawResults+12);
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupD(), i);
		stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageRawResults+18);

		uint16_t* realVoltage = (uint16_t*)voltageRawResults;
		printf("voltages for bank: %d\n", i);
		for(uint8_t j = 0; j < 12; j++){

				if(BMS_CELL_MAP[j] != -1){
					printf("Cell %d voltage: %f\n", j, realVoltage[j]*0.0000f);
				}
				// printf("BMS MAP VAL %d ", BMS_CELL_MAP[j]);
				// printf("Cell %d voltage: %x\n", j, realVoltage[j]);

		}
		printf("\n");
	}

}


void checkTemperatures(LTC681xParallelBus& ltcBusInterface){
	uint8_t rawTempVoltageReadings[60] = {0};
	// use the multiplexer to get all 6 sensors for the temps
	for(uint8_t i = 0; i < NUM_CELL_BANKS; i++){
		LTC6811::Configuration &config = chips[i].getConfig();

		for(uint8_t j = 0; j < NUM_CELLS_PER_BANK; j++){
     		//printf("number is %d\n", j);
	        if(j & 0b00000001){
	        	config.gpio1 = LTC6811::GPIOOutputState::kHigh;
	        }else{
	        	config.gpio1 = LTC6811::GPIOOutputState::kLow;
	        }
	        if((j & 0b00000010)>>1){
	        	config.gpio2 = LTC6811::GPIOOutputState::kHigh;
	        }else{
	        	config.gpio2 = LTC6811::GPIOOutputState::kLow;
	        }
	        if((j & 0b00000100)>>2){
	        	config.gpio3 = LTC6811::GPIOOutputState::kHigh;	
	        }else{
	        	config.gpio3 = LTC6811::GPIOOutputState::kLow;
	        }
	        config.gpio4 = LTC6811::GPIOOutputState::kPassive;

	       	chips[i].updateConfig();

	       	ltcBusInterface.WakeupBus();

	       	LTC681xBus::BusCommand command = LTC681xParallelBus::BuildAddressedBusCommand(StartGpioADC(AdcMode::k7k, GpioSelection::k4), i);
	       	//read voltage on gpio 4 which reads the tempsensor input forwarded by the multiplexer
	       	ltcBusInterface.SendCommand(command);

		}
		ThisThread::sleep_for(3ms);
		ltcBusInterface.WakeupBus();
		LTC681xBus::BusCommand command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupA(),i);
		ltcBusInterface.SendReadCommand(command, ((uint8_t*)rawTempVoltageReadings)+(i*12));
		command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupB(),i);
		ltcBusInterface.SendReadCommand(command, (uint8_t*)(rawTempVoltageReadings+(i*12)+6));


	}
	uint16_t realTempVoltages[30] = {0}; 

	
	printf("TempVoltage readings ... \n\n");
	for(uint8_t i = 0; i < 30; i++){
		if(i%5==0){
			printf("bank \n");
		}
		int8_t tempval = convertTemp(realTempVoltages[i]);
		printf("voltage: %d, temperature: %d\n", realTempVoltages[i], tempval);
	}



}







int main(){
	ThisThread::sleep_for(500ms);
	//printf("mainfunction\n");
	initalizeIO();
	LTC681xParallelBus ltcBusInterface(spiInterface);

	for(uint8_t i = 0; i < NUM_CELL_BANKS; i++){
		chips.push_back(LTC6811(ltcBusInterface, i));
	}


	// first step is to wakeup the bus. 
	LTC681xBus::LTC681xBusStatus stat = ltcBusInterface.WakeupBus();
	if(stat != LTC681xBus::LTC681xBusStatus::Ok){
		printf("unable to wakeup bus\n");
	}

	printf("Running self checks");

	selfChecks(ltcBusInterface);

	queue.call_every(2000, checkCellVoltages, ltcBusInterface);
	queue.call_every(2000, checkTemperatures, ltcBusInterface);
	//checkCellVoltages(ltcBusInterface);
	//checkTemperatures(ltcBusInterface); 
	queue.dispatch_forever();

	printf("\n\n\n");

	// printf("testing conversion thing......\n");

	// printf("testing value 2440 ... %d\n", convertTemp(2440));
	// printf("testing zero ..... %d\n", convertTemp(0000));




	return 0;
}