#include "tempPrecharge.h"



preCharger::preCharger(BMS* currentInstance){
	BMSInstance = currentInstance;
	preChargeStatus = PRECHARGE_IDLE;
}

bool preCharger::checkGLV(){

    float glvVoltage = (uint16_t)(BMSInstance->GLV_Voltage->read() * 3.3 * 21.9 / 3.9 * 1000);
    // CurrentBMSStatus.glv_voltage = (uint16_t)(glv * 1000);
    //  BMSInstance.bms_stat_message.glv_voltage = glvVoltage;
    return (
        glvVoltage > 11.0f && glvVoltage < 15.0f
    ); // place holder for 11 to 15 volts? we should find out the limit

}


float preCharger::readDcBusVoltage(){
	BMSInstance->CAN_POWERTRAIN->read(BMSInstance->msg);
    uint32_t id = BMSInstance.msg.id;
    unsigned char* data = BMSInstance->msg->data;
    if (!BMSInstance.Charge_State_Filtered.read()) {
        switch (id) { // copied over from fs3
        case 0x682:   // temperature message from MC
            dcBusVoltage = (data[2] | (data[3] << 8));
            break;
        default:
            break;
        }
    }
    return dcBusVoltage; // this is not correct has to be read from can, but we can use fs3
                         // implementation for now
}

bool preCharger::shutDownClosed(){
	return BMSInstance->Shutdown_Measure->read(); // ideally i dont think we need to read the IMD b/c it should open the shutdown circuit but....
}

bool preCharger::preChargeComplete(){

	return dcBusVoltage >= 0.9f * BMSInstance->totalBatteryVoltage; // i think this is good i wanna double check tho

}

void preCharger::checkPrecharge(){
	if(preChargeStatus == PRECHARGE_IDLE){
		if(BMSInstance->totalBatteryVoltage <= )
	}
}