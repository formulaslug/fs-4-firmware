#pragma once

#include "mbed.h"

struct VehicleState{
    uint8_t shutdown_final = 0;
    uint8_t SOC = 0;
    bool BMS_fault = false;
    bool IMD_fault = false;
    uint8_t shutdown_out;  //Two Shutdowns wonder which to use
    bool precharge_done = false; 
    bool precharging = false; 
    bool charging = false; 
    uint16_t pack_voltage = 0; 
    uint16_t glv_voltage = 0;
    uint32_t battery_fault_index = 0; 

    uint16_t DC_bus_voltage = 0;
    uint8_t controller_temp = 0;
    uint16_t SME_wheel_speed = 0;
    bool SME_controller_overtemp = false;
    bool SME_running = false;
    bool SME_powering_enabled = false;
    bool SME_powering_ready = false;
    uint8_t motor_temp = 0;
    uint8_t SME_fault_code = 0; 
    uint8_t SME_fault_level = 0; 

    bool ready_to_drive = false; 
    bool ETC_precharge_done = false;
    uint8_t brake_pedal_travel = 0;
    uint8_t BPPS_voltage = 0;
    bool implaus_APPS_out_of_range = false;
    bool implaus_BPPS_out_of_range = false;
    bool implaus_APPS_deviation = false;
    bool implaus_BSE_out_of_range = false;
    bool implaus_brake_and_accel = false;

    uint16_t GPS_speed = 0;
    uint8_t high_cell_temp = 0;
    bool fan_enabled = false;

    // Tire temperatures per corner
    uint8_t fl_side_temp = 0;
    uint8_t fl_tire_temps[8] = {0};
    uint8_t fr_side_temp = 0;
    uint8_t fr_tire_temps[8] = {0};
    uint8_t bl_side_temp = 0;
    uint8_t bl_tire_temps[8] = {0};
    uint8_t br_side_temp = 0;
    uint8_t br_tire_temps[8] = {0};
    // Wheelspeeds
    uint8_t fl_wheel_speed = 0;
    uint8_t fr_wheel_speed = 0;
    uint8_t bl_wheel_speed = 0;
    uint8_t br_wheel_speed = 0;

    // IZZE brake disc temperatures (°C, scaling applied)
    float izze_fl_brake_temp = 0;
    float izze_fr_brake_temp = 0;
    float izze_bl_brake_temp = 0;
    float izze_br_brake_temp = 0;

    // VCU brake line pressures (psi)
    uint16_t front_brake_pressure = 0;
    uint16_t rear_brake_pressure = 0;

    // Battery cell temperatures: 5 modules × 6 cells (°C, 0.2 scaling applied)
    uint8_t battery_mod_temps_A[5][6] = {0};
    uint8_t battery_mod_temps_B[5][6] = {0};

    // Battery cell voltages: 5 modules × 6 cells (raw byte)
    uint8_t battery_mod_voltages[5][6] = {0};

    uint8_t drive_state = 0;
    uint8_t traction_state = 0;
    uint8_t regen_state = 0; 
};
