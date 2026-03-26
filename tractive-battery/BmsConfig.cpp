#include "BmsConfig.h"

//literally just a constructor for the BMS object


BMS::BMS(){
	// //assume all is well at startup....
	AnalogIn V_Out_Positive = AnalogIn(PC_0);
    AnalogIn V_Out_Negative = AnalogIn(PC_1);
    DigitalIn Charge_State_Filtered = DigitalIn(PC_2); // assume 1 for charging 0 for not charging
    DigitalIn IMD_Fault_3V3 = DigitalIn(PC_4);
    DigitalOut nBMS_Fault_3V3 = DigitalOut(PC_5);
    PwmOut Fan_PWM = PwmOut(PC_8);
    DigitalOut TS_READY = DigitalOut(PC_9); //look more into this one (i think its a precharge indicator )
    DigitalIn Shutdown_In_3V3_Filtered = DigitalIn(PA_0); // status of the shutdown circuit before bms
    DigitalIn Shutdown_Out_3V3_Filtered = DigitalIn(PA_1); // status of the shutdown circuit after bms
    DigitalIn SH_RESET_3V3 = DigitalIn(PA_2); 
    DigitalIn Shutdown_Measure = DigitalIn(PA_6);
    AnalogIn GLV_Voltage = AnalogIn(PA_7);
    BufferedSerial VCP_UART = BufferedSerial(PA_9, PA_10); // some configuration for this needs to be done at startup see mbedosce
    CAN CAN_POWERTRAIN = CAN(PA_11, PA_12);
    DigitalOut nPrechargeControl = DigitalOut(PB_0);
    SPI spiInterface = SPI(PB_4, PB_5, PB_10, PB_9, use_gpio_ssel);
    DigitalOut TS1W_PU_Control = DigitalOut(PB_15);
    OneWire TS1W = OneWire(PB_14); // look up more on 1 wire interface 




    currentState = ACTIVE;

    bms_stat_message={
    	//assume everything is good at startup
    	false,
    	false,
    	false,
    	false,
    	false,
    	false,
    	false,
    	false,
    	false,
    	false,
    	false,
    	false,
    	false,
    	false,
    	0,
    	0,
    	0,
    	0
    };

    // create the ds18b20 interface

     


}