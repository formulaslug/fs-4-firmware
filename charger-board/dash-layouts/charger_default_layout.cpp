

#include "charger_default_layout.h"


void drawChargerDefaultLayout(BT817Q& display, bool is_charging) {
    display.startFrame();
    display.clear(0, 0, 0);
    if (is_charging) {
        display.drawText(240, 136, "Charging", green, 30, BT817Q::OPT_CENTER);
    } else {
        display.drawText(240, 136, "Not Charging", red, 30, BT817Q::OPT_CENTER);
    }
}
