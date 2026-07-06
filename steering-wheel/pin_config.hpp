#pragma once

#include "mbed.h"

//Pin Stuff
constexpr PinName PIN_MOSI = PA_7;
constexpr PinName PIN_MISO = PA_6;
constexpr PinName PIN_SCK = PA_5;
constexpr PinName PIN_CS = PA_4;
constexpr PinName PIN_PDN = PA_15;
constexpr PinName PIN_IRQ = PB_7;
constexpr PinName PIN_LAYOUT = PB_4;
constexpr PinName PIN_TC = PB_0; //ADC1_IN15, Got these values from custom_targets folder
constexpr PinName PIN_REGEN = PA_0; //ADC1_IN1
constexpr PinName PIN_MODE = PA_1; //ADC1_IN2
constexpr PinName PIN_CAN_RX = PA_11; //FDCAN1_RX
constexpr PinName PIN_CAN_TX = PA_12; //FDCAN1_TX

constexpr uint32_t CAN_FREQUENCY = 1000000; //1Mhz Can Frequency not sure if its 0.2Mhz or 2Mhz