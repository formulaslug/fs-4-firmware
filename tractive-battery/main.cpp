// #include "BMSFaultDetection.h"
#include "mbed.h"
#include "BMS.h"
#include "prechargeLogic.h"

/*

current need to dos:
1 wire for tray temp sensors sort of implemented - need more info - important - part of BMSFaultDetection - as of 4-4-26 still not enough info 
shutdown circuit monitoring - not implemented - part of BMSFaultDetection (go through shutdown sequence for battery but do not throw a fault, also open precharge relay )
current sensing - not implemented - part of telemetry
imd monitoring  - not implemented - part of telemetry

precharge - in progress........



Status of 1 wire:
cannot find info on the ids of the sensors, we can search for them however so.....

*/
// need to initialize everything on startup - assume everything is okay at first

bool eMeterPresent = false;

// EventQueue queue(5*EVENTS_EVENT_SIZE);


int main(){ 

	BMS BMSInstance;
	//intializing the uart stuff
	// this implementation is temporary
	BMSInstance.VCP_UART.set_baud(115200);


	for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
		for(uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++){
			LTC6810::TMP1075_Handle_t sens;
			sens.i2c_address = TMP1075_ADDRESSES[j];
			sens.temp_reg = 0x00; // it gives out 0 degrees until the first conversion so
			BMSInstance.sensors[i][j] = sens;
		}
			
	}

	// One wire section....
	/*
	we are using the ds18b20 sensors on the 1 wire bus, according to https://www.analog.com/en/resources/technical-articles/how-to-power-the-extended-features-of-1wire-devices.html 
	these sensors need a little extra power for temperature conversions.... when the emeter is connected to the car it is able to provide this power
	when the e meter is not connected to the car, a pmos pull up transistor is used to provide this extra power (I think).

	*/

	if(!eMeterPresent){
		BMSInstance.TS1W_PU_Control = 1;
	}else{
		BMSInstance.TS1W_PU_Control = 0;
	}



	//TEMPORARY: searching for 1 wire sensors......

	//should print out to serial the address of any one wire bus temp sensor is it the same as fs3? idk
	debug_search_for_ds18b20_address(BMSInstance.TS1W);



	for(uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS; i++){
		BMSInstance.trayTempSensors.push_back(DS18B20(BMSInstance.TS1W, TRAYTEMP_SENSOR_ADDRESSES[i]));
	}


	// char msg1[] = "Intialization complete\n";
	// BMSInstance.VCP_UART.write(msg1, sizeof(msg1));

	printf("Initialization complete\n");



	// BMSInstance.spiInterface.write(0xaa); //temporary debug 

	// controller(ltcBusInterface, BMSInstance);
	//period of this is temporary and arbitrary - ideally i would want this to run more often, the current period is for testing purposes 
	// queue.call_every(2000ms, &BMSInstance, &BMS::controller);
	// queue.dispatch_forever();


	return 0;
}