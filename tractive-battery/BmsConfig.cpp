#include "BmsConfig.h"

//literally just a constructor for the BMS object


BMS::BMS(){ 
    
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
    	-1,
    	-1,
    	-1
    };

    // create the ds18b20 interface




}