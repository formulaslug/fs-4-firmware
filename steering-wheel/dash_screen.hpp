//
// Created by Nova Mondal on 5/12/25.
//

#ifndef LAYOUTS_H
#define LAYOUTS_H

#include "BT817Q.hpp"
#include "vehicle_state.hpp"

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

    void draw_thermal_screen(VehicleState& state);

    void draw_debug_fault_layout(
            VehicleState& state,
            uint16_t fps,
            int tick
    );

    void draw_main_display(VehicleState& state);

    void debug_cell_temps(VehicleState& state);

    void debug_cell_voltages(VehicleState& state);

    void draw_temp_cell(uint16_t x, uint16_t y, uint8_t temp);

    void draw_voltage_cell(uint16_t x, uint16_t y, uint16_t voltage);

    Color cell_temp_to_color(uint8_t temp);

    Color cell_voltage_to_color(uint16_t voltage);

    Color SOC_to_color(uint8_t SOC);

    static constexpr uint8_t ACC_WARNING_VOLT = 94;
    static constexpr uint8_t ACC_WARNING2 = 100;
    static constexpr uint8_t ACC_WARNING3 = 103;
    static constexpr uint8_t ACC_WARNING_TEMP = 54;

    static constexpr uint8_t CELL_WARNING_TEMP = 50;
    static constexpr uint8_t CELL_NORMAL_TEMP = 45;
    static constexpr uint8_t CELL_LOW_TEMP = 15;

    static constexpr uint16_t CELL_HIGH_VOLT = 4000;
    static constexpr uint16_t CELL_STD_VOLT = 3400;
    static constexpr uint16_t CELL_LOW_VOLT = 2800;

    static constexpr uint16_t PACK_STD_VOLT = 10300;
    static constexpr uint16_t PACK_WARNING_VOLT = 10000;
    static constexpr uint16_t PACK_FLASH_VOLT = 9400;

    static constexpr uint16_t CELL_WIDTH = 100;
    static constexpr uint16_t CELL_HEIGHT = 68;
    static constexpr uint16_t CELL_FONT = 24;
};

#endif // LAYOUTS_H
