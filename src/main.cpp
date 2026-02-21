// #include "BmsConfig.h"
#include "LTC681xBus.h"
#include "LTC681xParallelBus.h"
#include "LTC681xCommand.h"
// #include "LTC6811.h"
#include "LTC6810.h"
#include "mbed.h"
#include "BMSFaultDetection.h"

SPI* spiInterface;



int main(){

	spiInterface = new SPI(SPI_MOSI, SPI_MISO, CLK, BMSCS, use_gpio_ssel);
	spiInterface->format(8,0);
	// std::vector<LTC6810> chips;
	LTC681xParallelBus ltcBusInterface(spiInterface);

	for(uint8_t i = 0; i < BATTERYBANKS; i++){
		chips.push_back(LTC6810(ltcBusInterface, i));
	}
	for(uint8_t i = 0; i < BATTERYBANKS; i++){
		LTC6810::TMP1075_Handle_t sens;
		sens.i2c_address = 0x00;
		sens.temp_reg = 0x00;
		sensors[i] = sens;
	}

	readCellVoltages(ltcBusInterface);


	return 0;
}