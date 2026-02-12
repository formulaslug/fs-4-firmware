#ifndef CONFIG_H
#define CONFIG_H

//Should probably move these structs and function initializations somewhere else but idk where

/*
Corner that the current board is in, determined by dip switches
Just for organization in determining the can message id
*/
enum Corner {
    FR,
    FL,
    BR,
    BL
};
/*
Can message id's, currently using the FS-3 message id's
*/
struct cornerConfig {
    uint16_t tpdo_data_id;
    uint16_t tpdo_tiretemp_id;
    bool has_tiretemp_1x8;
    bool has_tiretemp_1x1;
};
/*
Reads the corner that the board is on based on dip switch configuration
*/
Corner readCorner();
/*
Gets the appropriate can message id for the corner that the board is on
*/
cornerConfig getCornerConfig(Corner pos);


#ifndef PIN_CAN1_RX
#define PIN_CAN1_RX PA_11
#endif

#ifndef PIN_CAN1_TX
#define PIN_CAN1_TX PA_12
#endif

#ifndef CAN_FREQUENCY
#define CAN_FREQUENCY 500000 //Don't know/not sure the values with comments
#endif

#ifndef PIN_WHEEL_SENSOR
#define PIN_WHEEL_SENSOR PA_5 
#endif

#ifndef TEETH
#define TEETH 50 //Dunno
#endif

#ifndef TIRE_CIRCUMFERENCE
#define TIRE_CIRCUMFERENCE 50 //Dunno
#endif

#ifndef PIN_DIP_1
#define PIN_DIP_1 PA_0 //Dunno
#endif

#ifndef PIN_DIP_2
#define PIN_DIP_2 PA_1 //Dunno
#endif

#ifndef PIN_SUSPENSION
#define PIN_SUSPENSION PA_4
#endif

#ifndef I2C_SDA
#define I2C_SDA PA_6 //Dunno
#endif

#ifndef I2C_SCL
#define I2C_SCL PA_7 //Dunno
#endif

#endif //CONFIG_H
