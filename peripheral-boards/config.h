#ifndef CONFIG_H
#define CONFIG_H

//Should probably move the structs and function initializations somewhere else but I'm not sure where

/*
Corner that the current board is in, determined by dip switches
This isn't really needed, just for organization in determining the can message id
*/
enum Corner {
    FR,
    FL,
    BR,
    BL
};
/*
Can message id's
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
#define CAN_FREQUENCY 500000 //Have no Idea
#endif

#ifndef PIN_WHEEL_SENSOR
#define PIN_WHEEL_SENSOR PA_5
#endif

#ifndef TEETH
#define TEETH 50 //Don't know, random number
#endif

#ifndef TIRE_CIRCUMFERENCE
#define TIRE_CIRCUMFERENCE 50 //Don't know, random number
#endif

#ifndef PIN_DIP_1
#define PIN_DIP_1 PA_0 //Check these, not exactly sure if they are the correct switch pins
#endif

#ifndef PIN_DIP_2
#define PIN_DIP_2 PA_1 //Check these, not exactly sure if they are the correct switch pins
#endif

#ifndef PIN_SUSPENSION
#define PIN_SUSPENSION PA_4
#endif

#ifndef I2C_SDA
#define I2C_SDA PA_6 //Have no idea
#endif

#ifndef I2C_SCL
#define I2C_SCL PA_7 //Have no idea
#endif


#endif //CONFIG_H
