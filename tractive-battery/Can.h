// Copyright (c) 2018 Formula Slug. All Rights Reserved.
#ifndef CAN_H
#define CAN_H

#include "mbed.h"
#include "BMS.h"

// Message sent (BMS to car)
#define ID_ACC_STATUS        0x188
#define ID_ACC_POWER         0x288
#define ID_ACC_VOLTS_BASE    0x400
#define ID_ACC_TEMPS_BASE    0x500
#define ID_ACC_TRAY_TEMPS    0x388

// Messaged recieved (car to BMS)
#define ID_SMPC_CHARGE_CTRL  0x206how 

// DBC Scaling Constants
#define VOLT_OFFSET 2.0f
#define VOLT_SCALE  100.0f

// Send battery info to car
void canSend(BMS &bms);

// Look for and recieve the messages coming in
void canReceive(BMS &bms);

#endif
