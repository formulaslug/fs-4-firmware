#pragma once

#include "mbed.h"

struct VehicleState{
    uint8_t batt_status_shutdown_final;
    uint8_t batt_power_soc;
    uint8_t batt_status_bms_fault;
    uint8_t batt_status_imd_fault;
    uint8_t batt_status_shutdown_out;  //Two Shutdowns wonder which to use
    uint8_t batt_status_precharge_done; 
    uint8_t batt_status_precharging; 
    uint8_t batt_status_charging; 
    uint16_t batt_power_pack_voltage; 
    uint16_t batt_status_glv_voltage;
    uint32_t batt_status_fault_index; 
    uint16_t sme_temp_dc_bus_v;
    uint8_t sme_temp_controllertemperature;
    uint16_t sme_trqspd_speed;
    uint8_t sme_trqspd_controller_overtermp;
    uint8_t sme_trqspd_running;
    uint8_t sme_trqspd_powering_enabled;
    uint8_t sme_trqspd_powering_ready;
    uint8_t sme_temp_motortemperature;
    uint8_t sme_temp_faultcode; 
    uint8_t sme_temp_faultlevel; 
    uint8_t vcu_etc_ready_to_drive; 
    uint8_t vcu_etc_batt_precharge_done;
    uint8_t vcu_etc_brake_pedal_travel; 
    uint8_t vcu_etc_bpps_he;
    uint8_t vcu_etc_implausibility_apps_out_of_range;
    uint8_t vcu_etc_implausibility_bpps_out_of_range;
    uint8_t vcu_etc_implausibility_apps_deviation;
    uint8_t vcu_etc_implausibility_bse_out_of_range;
    uint8_t vcu_etc_implausibility_brake_and_accel;
    uint16_t vdm_gps_speed;
    uint8_t batt_stat_high_cell_temp;

    // Tire temperatures per corner
    uint8_t tperiph_fl_side_temp;
    uint8_t tperiph_fl_tire_temps[8];
    uint8_t tperiph_fr_side_temp;
    uint8_t tperiph_fr_tire_temps[8];
    uint8_t tperiph_bl_side_temp;
    uint8_t tperiph_bl_tire_temps[8];
    uint8_t tperiph_br_side_temp;
    uint8_t tperiph_br_tire_temps[8];

    // IZZE brake disc temperatures (°C, scaling applied)
    float izze_brake_fl;
    float izze_brake_fr;
    float izze_brake_rl;
    float izze_brake_rr;

    // VCU brake line pressures (psi)
    uint16_t vcu_etc_brake_pressure_front;
    uint16_t vcu_etc_brake_pressure_rear;

    // Battery cell temperatures: 5 modules × 6 cells (°C, 0.2 scaling applied)
    uint8_t batt_mod_temps[5][6];

    // Battery cell voltages: 5 modules × 6 cells (raw byte)
    uint8_t batt_mod_volts[5][6];

};

void readLayoutButton();
void readDials();
void drawScreenLayout();
void canISR();
void processCANMessage();
void sendCANMessage();