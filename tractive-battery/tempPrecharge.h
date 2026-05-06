#ifndef TEMPPRECHARGE_H
#define TEMPPRECHARGE_H

#include "BMS.h"
#include "mbed.h"



class preCharger(){
private:
	BMS *BMSInstance;
public:
	enum preChargeStatus{
		PRECHARGE_IDLE = 0,
		PRECHARGE_ACTIVE = 1,
		PRECHARGE_COMPLETE = 2 
	};
	preChargeStatus currentStatus;
	preCharger(BMS*);

	void checkPrecharge();
	bool checkGLV();
	float readDcBusVoltage();
	bool shutDownClosed();
	bool preChargeComplete();
	


}



#endif