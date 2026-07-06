#pragma once

#include "BT817Q.hpp"
void drawChargerDefaultLayout(BT817Q& display,
                                bool is_charging,
                            float pack_voltage,
                            uint16_t soc,
                            float current,
                            float min_temp,
                            float max_temp,
                            float avg_temp,
                            float energy_used,
                            uint32_t charge_time);

