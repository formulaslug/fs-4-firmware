//
// Created by Nova Mondal on 5/12/25.
//

#ifndef LAYOUTS_H
#define LAYOUTS_H

#include "BT817Q.hpp"

struct Faults {
    bool fans : 1;
    bool precharge : 1;
    bool shutdown : 1;
};

class DashScreen : public BT817Q {
public:
    DashScreen(
        PinName mosi,
        PinName miso,
        PinName sck,
        PinName cs,
        PinName pdn,
        PinName irq,
        EvePanel panel
    );

    void drawThermalScreen(
        uint8_t acc_temp, uint8_t mtr_temp, uint8_t ctrl_temp,
        float fl_surface, float fl_side,
        float fr_surface, float fr_side,
        float rl_surface, float rl_side,
        float rr_surface, float rr_side,
        float brake_fl, float brake_fr, float brake_rl, float brake_rr,
        float brake_f, float brake_r
    );

    void drawDebugFaultLayout(
        uint8_t bms,           // BMS_FAULT
        uint8_t imd,           // IMD_FAULT
        uint8_t sdwn,          // SHUTDOWN
        uint8_t pchgd,         // PRECHARGED
        uint8_t pchgi,         // PRECHARGING
        uint8_t chging,        // CHARGING
        uint16_t packv,        // PACK_VOLTAGE
        uint16_t glv,          // GROUNDED LOW VOLTAGE
        uint32_t cellfault,    // CELL FAULT INDEX
        uint8_t rtd,           // READY TO DRIVE
        uint8_t implausiblity, // BRAKE PEDAL SYSTEM IMPLAUSIBILITY
        uint8_t tsactive,      // TRACTICE SYSTEM ACTIVE
        uint8_t pedaltravel,   // PEDAL TRAVEL PERCENT
        uint8_t brakesensev,   // BRAKE SENSOR VOLTAGE
        uint8_t ctrlovertemp,  // CONTROLLER OVERTEMP
        uint8_t running,       // RUNNING
        uint8_t poweron,       // POWER ON
        uint8_t powerrdy,      // POWER READY
        uint8_t motortemp,     // MOTOR TEMP
        uint8_t faultcode,     // FAULT CODE
        uint8_t faultlevel,    // FAULT LEVEL
        uint16_t fps,
        //uint8_t wheelspeedFL,
        int tick
    );

    void drawMainDisplay(
        bool shtd,
        bool mtr_ctrl,
        bool rtd,
        bool pchg,
        bool fans,
        uint16_t acc_volt,
        uint8_t acc_temp,
        uint8_t soc,
        int tick,
        uint16_t speed,
        const char* lap_time,
        uint16_t glv,
        uint8_t mtr_temp,
        uint8_t ctrl_temp,
        uint16_t dc_bus,
        uint8_t regen_state
    );

    void debugCellTemps(const uint8_t seg_temps_A[5][6], const uint8_t seg_temps_B[5][6]);

    void debugCellVolts(const uint8_t seg_volts[5][6]);

    void drawTempCell(uint16_t x, uint16_t y, uint8_t temp);

    void drawVoltCell(uint16_t x, uint16_t y, uint16_t volt);

    Color cellTempToColor(uint8_t temp);

    Color cellVoltToColor(uint16_t volt);

    Color socToColor(uint8_t soc);
};

#endif // LAYOUTS_H
