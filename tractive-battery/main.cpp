#include "BMSFaultDetection.h"
#include "mbed.h"

int main(){
	BMS BMSInstance;

	LTC681xParallelBus ltcBusInterface(&BMSInstance.spiInterface);



	for(uint8_t i = 0; i < NUM_BATTERY_MODULES; i++){
		for(uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++){
			LTC6810::TMP1075_Handle_t sens;
			sens.i2c_address = TMP1075_ADDRESSES[j];
			sens.temp_reg = 0x00; // it gives out 0 degrees until the first conversion so
			BMSInstance.sensors[i][j] = sens;
		}
			
	}

	// find out wether or not we are charging
	if(BMSInstance.Charge_State_Filtered.read()){
		BMSInstance.currentState = BMSInstance.CHARGING;
	}else{
		BMSInstance.currentState = BMSInstance.ACTIVE;
	}

	return 0;
}