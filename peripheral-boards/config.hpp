#pragma once

#include "mbed.h"

// Might want to move these structs and function initializations somewhere else but idk where

/*
Corner that the current board is in, determined by dip switches
Just for organization in determining the board location
*/
enum class Corner : uint8_t {
    FL,
    FR,
    BL,
    BR
};

/*
Can message id's, currently using the FS-3 message id's
*/
struct CornerConfig {
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
CornerConfig getCornerConfig(Corner pos);

/*
Generates and writes the CAN message for temperature data
*/
void sendCANtemp();
/*
Generates and writes the CAN message for speed, suspension, strain, and temp
*/
void sendCANtpdo();

void sendLastMessageTicks();

constexpr PinName PIN_STRAIN = PB_0;
constexpr PinName PIN_DIP_0 = PA_0;
constexpr PinName PIN_DIP_1 = PA_1;
constexpr PinName PIN_WHEEL_SENSOR = PA_5;
constexpr PinName PIN_SUSPENSION = PA_7;
constexpr PinName PIN_I2C2_SDA = PA_8;
constexpr PinName PIN_I2C2_SCL = PA_9;
constexpr PinName PIN_CAN1_RX = PA_11;
constexpr PinName PIN_CAN1_TX = PA_12;
constexpr PinName PIN_I2C1_SDA = PB_7;
constexpr PinName PIN_I2C1_SCL = PA_13;

constexpr bool ok8 = true; // To manually disable temp sensor if they fail
constexpr bool ok1 = true;

constexpr uint32_t CAN_FREQUENCY = 2000000; //2Mhz
constexpr uint8_t TEETH_PER_REV = 60; // Double check this
