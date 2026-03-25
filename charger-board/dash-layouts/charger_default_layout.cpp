

#include "charger_default_layout.h"


void drawChargerDefaultLayout(BT817Q& display,
                                bool is_charging,
                            float pack_voltage,
                            uint16_t soc,
                            float current,
                            float min_temp,
                            float max_temp,
                            float avg_temp,
                            float energy_used,
                            uint32_t charge_time) {
    display.startFrame();
    display.clear(0, 0, 0);
    if (is_charging) {
        display.drawText(400, 20, "CHARGING", green, 30, BT817Q::OPT_CENTER);
    } else {
        display.drawText(400, 20, "NOT CHARGING", red, 30, BT817Q::OPT_CENTER);
    }
    //space for future live graphs
    //Voltage graph
    display.drawRect(Point{20, 60}, Point{460, 175}, white);
    display.drawText(35, 75, "Voltage", white, 24, 0);
    display.drawFormattedText(240, 110, "$0.1f V", white, 28, BT817Q::OPT_CENTER, pack_voltage);
    //SOC graph
     display.drawRect(Point{20, 160}, Point{460, 245}, white);
    display.drawText(35, 175, "State of Charge", white, 24, 0);
    display.drawFormattedText(240, 210, "%d %%", white, 28, BT817Q::OPT_CENTER, soc);

    //current graph
    display.drawRect(Point{20, 260}, Point{460, 345}, white);
    display.drawText(35, 275, "Current Limit", white, 24, 0);
    display.drawFormattedText(240, 310, "%0.1f A", white, 28, BT817Q::OPT_CENTER, current);\
    
    //temp grap
    display.drawRect(Point{20, 360}, Point{460, 445}, white);
    display.drawText(35, 375, "Temperature", white, 24, 0);
    display.drawFormattedText(240, 410, "%0.1f C", white, 28, BT817Q::OPT_CENTER, max_temp);

    //Stats column right side of screen
    display.drawFormattedText(520, 100, "Min Temp: %0.1f C", white, 24, 0, min_temp);
    display.drawFormattedText(520, 155, "Avg Temp: %0.1f C", white, 24, 0, avg_temp);
    display.drawFormattedText(520, 210, "Max Temp: %0.1f C", white, 24, 0, max_temp);
    display.drawFormattedText(520, 265, "Energy Used: %0.1f", white, 24, 0, energy_used);
    display.drawFormattedText(520, 320, "Charge Time: %lu min", white, 24, 0,
                              static_cast<unsigned long>(charge_time));

    display.endFrame();


}
