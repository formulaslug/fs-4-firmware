// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#ifndef _FS_BMS_SRC_CAN_H_
#define _FS_BMS_SRC_CAN_H_

#include "mbed.h"
#include "BmsConfig.h" //I added this inclusion
#include <cstdint>

// Message sent out (BMS to car)
#define ID_ACC_STATUS         392   // 0x188
#define ID_ACC_POWER          648   // 0x288
#define ID_ACC_VOLTS_BASE     401   // 0x191
#define ID_ACC_TEMPS_BASE     657   // 0x291
#define ID_ACC_TRAY_TEMPS     2147484552 // 0x80000184 (29-bit extended)

// Incoming messages
#define ID_SMPC_CHARGE_CTRL   518   // 0x206
#define ID_VDM_CONTROL        512   // 0x200

// DBC Scaling Constants
#define VOLT_OFFSET 2.0f
#define VOLT_SCALE  100.0f

// Send battery info to car
void canSend(status_msg* s, tray_temps_msg* t, uint16_t packV, uint8_t soc, 
             int16_t curr, uint8_t fan, uint16_t* v_arr, int8_t* t_arr);

// Look for and recieve the messages coming in
void canReceive();

#endif
