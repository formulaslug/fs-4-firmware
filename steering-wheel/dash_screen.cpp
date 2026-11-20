//
// Created by Nova Mondal on 5/12/25.
// Edited for FS-4 by Sid Shastri on 2/23/26
#include "dash_screen.hpp"
#include "mbed.h"
#include "vehicle_state.hpp"
#include <cstdio>
#include <string>

DashScreen::DashScreen(PinName mosi, PinName miso, PinName sck, PinName cs, PinName pdn, PinName irq, EvePanel panel)
    : BT817Q(mosi, miso, sck, cs, pdn, irq, panel) {}

void DashScreen::draw_thermal_screen(VehicleState& state) {
    if (failure == start_frame()) return;

    clear(255, 255, 255);
    load_fonts();

    auto set_temp_color = [&](int temp, int warn_temp, int red_temp) {
        if (temp >= red_temp) {
            set_main_color(red);
        } else if (temp >= warn_temp) {
            set_main_color(yellow);
        } else {
            set_main_color(green);
        }
    };

    // top row
    const int top_Y = 15;
    const int top_H = 70;
    const int top_W = 230;
    const int top_spacing = 20;
    int curr_X = 35;

    auto draw_top_box = [&](const char* label, uint8_t val, int x, int warn_temp, int red_temp) {
        if (val >= red_temp) {
            set_main_color(red);
        } else if (val >= warn_temp) {
            set_main_color(yellow);
        } else {
            set_main_color(green);
        }

        draw_rect({(uint16_t)x, (uint16_t)top_Y}, {(uint16_t)(x + top_W), (uint16_t)(top_Y + top_H)});
        set_main_color(black);
        draw_formatted_text(x + top_W / 2, top_Y + top_H / 2, label, 28, OPT_CENTER, val);
    };

    set_main_color((state.high_cell_temp > ACC_WARNING_TEMP) ? red : green);
    draw_rect({(uint16_t)curr_X, (uint16_t)top_Y}, {(uint16_t)(curr_X + top_W), (uint16_t)(top_Y + top_H)});
    set_main_color(black);
    draw_formatted_text(curr_X + top_W / 2, top_Y + top_H / 2, "ACC: %dC", 28, OPT_CENTER, state.high_cell_temp);
    curr_X += top_W + top_spacing;

    draw_top_box("MTR: %dC", state.motor_temp, curr_X, 80, 100);
    curr_X += top_W + top_spacing;

    draw_top_box("CTRL: %dC", state.controller_temp, curr_X, 70, 85);

    // main area
    set_main_color(black);
    draw_rect({30, 100}, {770, 470}, 3);

    const int box_W = 120;
    const int box_H = 135;
    const int horizontal_gap = 140;
    const int vertical_gap = 40;
    const int text_line_height = 35;

    int left_box_X = 400 - (horizontal_gap / 2) - box_W;
    int right_box_X = 400 + (horizontal_gap / 2);

    auto draw_wheel_pod = [&](int x, int y, float surface, float side, float brk,
                            const char* label, bool is_left) {
        int avg_value = (int)((surface + side) / 2.0f);

        set_temp_color(avg_value, 80, 100);
        draw_rect({(uint16_t)x, (uint16_t)y}, {(uint16_t)(x + box_W), (uint16_t)(y + box_H)});

        // inner boxes
        set_main_color(black);
        draw_formatted_text(x + box_W / 2, y + box_H / 2 - 25, label, 30, OPT_CENTER);
        draw_formatted_text(x + box_W / 2, y + box_H / 2 + 25, "%dC", 30, OPT_CENTER, avg_value);

        // side text
        int text_X = is_left ? (x - 25) : (x + box_W + 25);
        uint16_t align = is_left ? OPT_RIGHTX : 0;
        int text_start_Y = y + 15;

        set_temp_color(surface, 80, 100);
        draw_formatted_text(text_X, text_start_Y, "Surface: %dC", 24, align, surface);

        set_temp_color(side, 80, 100);
        draw_formatted_text(text_X, text_start_Y + text_line_height, "Side: %dC", 24, align, side);

        set_temp_color(brk, 80, 100);
        draw_formatted_text(text_X, text_start_Y + (text_line_height * 2), "Brk Disc: %dC", 24, align, brk);
    };

    int front_Y = 130;
    int rear_Y = front_Y + box_H + vertical_gap;

    auto avg_tire_temps = [](const uint8_t temps[8]) -> float {
        uint16_t sum = 0;
        for (int i = 0; i < 8; i++)
            sum += temps[i];
        return sum / 8.0f;
    };

    draw_wheel_pod(left_box_X, front_Y, avg_tire_temps(state.fl_tire_temps), state.fl_side_temp / 10.0f, state.izze_fl_brake_temp, "FL:", true);
    draw_wheel_pod(left_box_X, front_Y, avg_tire_temps(state.fr_tire_temps), state.fr_side_temp / 10.0f, state.izze_fr_brake_temp, "FR:", true);
    draw_wheel_pod(left_box_X, front_Y, avg_tire_temps(state.bl_tire_temps), state.bl_side_temp / 10.0f, state.izze_bl_brake_temp, "BL:", true);
    draw_wheel_pod(left_box_X, front_Y, avg_tire_temps(state.br_tire_temps), state.br_side_temp / 10.0f, state.izze_br_brake_temp, "BR:", true);

    // brake balance
    int brake_balance_f = 50;
    int brake_balance_r = 50;

    if ((state.front_brake_pressure + state.rear_brake_pressure) > 0.1f) {
        brake_balance_f = (int)(100 * (state.front_brake_pressure / (state.front_brake_pressure + state.rear_brake_pressure)));
        brake_balance_r = 100 - brake_balance_f;
    }

    set_main_color(black);
    draw_formatted_text(400, 270, "Brake Balance:", 30, OPT_CENTER);
    draw_formatted_text(400, 300, "%02d/%02d", 30, OPT_CENTER, brake_balance_f, brake_balance_r);

    end_frame();
}
void DashScreen::draw_debug_fault_layout(
        VehicleState& state
        uint16_t FPS,
        int tick
) {

    if (failure == start_frame()) {
        return;
    }
    //-----------\init-------------
    clear(255, 255, 255); // white background for frame why doesnt clear take a color I don't know
    load_fonts();          // Not sure what possible fonts there are to use, above around 17?

    // --- Column layout ---
    const uint16_t col1_x = 50;
    const uint16_t col2_x = 310;
    const uint16_t col3_x = 575;
    const uint16_t row_start = 80;
    const uint16_t row_step = 40;
    const uint16_t text_offset = 130;
    const uint16_t status_box_size = 16;
    const uint16_t status_box_offset = 190;
    const uint16_t font_height_adjustment = 5;

    // --- Section titles ---
    set_main_color(black);
    draw_text(col1_x, 40, "BAT", 24); // Battery
    draw_text(col2_x, 40, "ETC", 24); // Eletronic throttle control
    draw_text(col3_x, 40, "SME", 24); // The SME motor controller
    // Seperating lines between columns
    draw_line(Point{col1_x + status_box_offset + 24, 0}, Point{col1_x + status_box_offset + 24, 480}, 10);
    draw_line(Point{col2_x + status_box_offset + 24, 0}, Point{col2_x + status_box_offset + 24, 480}, 10);

    // --- Battery / Accumulator Column ---
    const char* battery_labels[] = {
        "BMS FLT", "IMD FLT", "SHDWN", "PRECHGD",
        "PRECHGING", "PACK V", "GLV", "CELL FLT"
    };
#define GLV_LOW 10800
#define GLV_HIGH 13500
#define MTR_TEMP_HIGH 80
    char battery_values[9][16];
    snprintf(battery_values[0], sizeof(battery_values[0]), "%s", state.BMS_fault ? "[FAULT]" : "[NO]");
    snprintf(battery_values[1], sizeof(battery_values[1]), "%s", state.IMD_fault ? "[FAULT]" : "[NO]");
    snprintf(battery_values[2], sizeof(battery_values[2]), "%s", state.shutdown_out ? "[TRUE]" : "[FALSE]");
    snprintf(battery_values[3], sizeof(battery_values[3]), "%s", state.precharge_done ? "[TRUE]" : "[FALSE]");
    snprintf(battery_values[4], sizeof(battery_values[4]), "%s", state.precharging ? "[TRUE]" : "[FALSE]");
    snprintf(battery_values[5], sizeof(battery_values[5]), "%u.%02u V", state.pack_voltage / 100, state.pack_voltage % 100);
    snprintf(battery_values[6], sizeof(battery_values[6]), "%u mV", state.glv_voltage);
    snprintf(battery_values[7], sizeof(battery_values[7]), "%lu", (unsigned long)state.battery_fault_index);
    for (uint16_t row = 0; row < 8; row++) {
        uint16_t element = row_start + row * row_step;
        Color box_color = green; // default color to green
        if (row == 5) {
            // Flash the box to alarm driver if MC volts below warning point
            if (state.pack_voltage / 100 < ACC_WARNING_VOLT) {
                if (tick % 2 == 0) {
                    box_color = red;
                } else {
                    box_color = yellow;
                }
            } else if (state.pack_voltage / 100 < ACC_WARNING2) {
                box_color = red;
            } else if (state.pack_voltage / 100 < ACC_WARNING3) {
                box_color = orange;
            }
        } else if (row == 6) {
            // GLV range checking
            if (state.glv_voltage > GLV_LOW && glv < GLV_HIGH) {
                box_color = green;
            } else {
                box_color = red;
            }
        } else if (row == 7) {
            // CELL FLT
            if (state.battery_fault_index != 0) {
                box_color = red;
            } else {
                box_color = green;
            }
        } else if (strstr(battery_values[row], "FAULT")) {
            box_color = red;
        } else if (strstr(battery_values[row], "FALSE")) {
            box_color = mid_gray;
        }

        set_main_color(box_color);
        draw_text(col1_x - 30, element, battery_labels[row], 23, OPT_CENTERY);
        draw_text(col1_x + text_offset, element, battery_values[row], 23);
        draw_rect(Point{col1_x + status_box_offset, static_cast<unsigned short>(element - font_height_adjustment)}, Point{col1_x + status_box_offset + status_box_size, static_cast<unsigned short>(element + status_box_size - font_height_adjustment)}, box_color);
        set_main_color(mid_gray);
    }

    // --- ETC Column ---
    const char* ETC_labels[] = {
        "RTD", "IMPLAUS", "TS ACTIVE", "PEDL TRVL", "BRAKE V"
    };
    char ETC_values[5][16];
    snprintf(ETC_values[0], sizeof(ETC_values[0]), "%s", state.ready_to_drive ? "[TRUE]" : "[FALSE]"); 
    snprintf(ETC_values[1], sizeof(ETC_values[1]), "%s", (state.implaus_APPS_deviation | state.implaus_APPS_out_of_range | state.implaus_BPPS_out_of_range | state.implaus_BSE_out_of_range | state.implaus_brake_and_accel) ? "[FAULT]" : "[NO]"); 
    snprintf(ETC_values[2], sizeof(ETC_values[2]), "%s", state.precharge_done ? "[TRUE]" : "[FALSE]"); 
    snprintf(ETC_values[3], sizeof(ETC_values[3]), "%u%%", state.brake_pedal_travel);
    snprintf(ETC_values[4], sizeof(ETC_values[4]), "%u.%01u V", state.BPPS_voltage / 10, state.BPPS_voltage % 10);
    for (uint16_t row = 0; row < 5; row++) {
        uint16_t element = row_start + row * row_step;
        Color box_color = green;
        if (row == 3) {
            // Not sure about ranges for pedal travel
            box_color = mid_gray;
        } else if (row == 4) {
            // Not sure about ranges for brake sensor voltage
            box_color = mid_gray;
        } else if (strstr(ETC_values[row], "FAULT"))
            box_color = red;
        else if (strstr(ETC_values[row], "FALSE"))
            box_color = mid_gray;
        set_main_color(box_color);
        draw_text(col2_x - 30, element, ETC_labels[row], 23, OPT_CENTERY);
        draw_text(col2_x + text_offset, element, ETC_values[row], 23);
        draw_rect(Point{col2_x + status_box_offset, static_cast<unsigned short>(element - font_height_adjustment)}, Point{col2_x + status_box_offset + status_box_size, static_cast<unsigned short>(element + status_box_size - font_height_adjustment)}, box_color);
        set_main_color(mid_gray);
    }

    // --- SME Column ---
    const char* SME_labels[] = {
        "CTRL OVR TEMP", "RUNNING", "PWR ON", "PWR RDY",
        "MOTR TEMP", "FLT CODE", "FLT LEVEL"
    };
    char SME_values[7][16];
    snprintf(SME_values[0], sizeof(SME_values[0]), "%s", state.SME_controller_overtemp ? "[TRUE]" : "[FALSE]");
    snprintf(SME_values[1], sizeof(SME_values[1]), "%s", state.SME_running ? "[TRUE]" : "[FALSE]");
    snprintf(SME_values[2], sizeof(SME_values[2]), "%s", state.SME_powering_enabled ? "[TRUE]" : "[FALSE]");
    snprintf(SME_values[3], sizeof(SME_values[3]), "%s", state.SME_powering_ready ? "[TRUE]" : "[FALSE]");
    snprintf(SME_values[4], sizeof(SME_values[4]), "%u.%01u 'C", state.motor_temp / 10, state.motor_temp % 10);
    snprintf(SME_values[5], sizeof(SME_values[5]), "%u", state.SME_fault_code);
    snprintf(SME_values[6], sizeof(SME_values[6]), "%u", state.SME_fault_level);
    for (uint16_t row = 0; row < 7; ++row) {
        uint16_t element = row_start + row * row_step;
        Color box_color = green;
        if (row == 4) {
            // motor temp
            if (state.motor_temp < MTR_TEMP_HIGH) {
                box_color = green;
            } else {
                box_color = red;
            }
        } else if (row == 5) {
            // fault code
            if (state.SME_fault_code <= 0) {
                box_color = green;
            } else {
                box_color = red;
            }
        } else if (row == 6) {
            // fault level
            if (state.SME_fault_level != 0) {
                box_color = red;
            } else {
                box_color = green;
            }
        } else if (strstr(SME_values[row], "FAULT")) {
            box_color = red;
        } else if (strstr(SME_values[row], "FALSE")) {
            box_color = mid_gray;
        }
        set_main_color(box_color);
        draw_text(col3_x - 30, element, SME_labels[row], 23, OPT_CENTERY);
        draw_text(col3_x + text_offset, element, SME_values[row], 23);
        draw_rect(Point{col3_x + status_box_offset, static_cast<unsigned short>(element - font_height_adjustment)}, Point{col3_x + status_box_offset + status_box_size, static_cast<unsigned short>(element + status_box_size - font_height_adjustment)}, box_color);
        set_main_color(mid_gray);
    }

    // String interpretations of common fault codes
    const uint16_t pos_X = 530;
    const uint16_t pos_Y = 430;
    draw_text(pos_X, 400, "MTR CTL: ", 23, OPT_CENTERY);
    string fault_name = "";
    if (state.SME_fault_code <= 0) {
        fault_name = "No Faults";
    } else {
        switch (state.SME_fault_code) {
            case 1:
                fault_name = "Over Voltage";
                break;
            case 2:
                fault_name = "Under Voltage";
                break;
            case 3:
                fault_name = "User Over Voltage";
                break;
            case 26:
                fault_name = "High Voltage";
                break;
            case 25:
                fault_name = "Low Voltage";
                break;
            case 62:
                fault_name = "Heartbeat Timeout";
                break;
            case 63:
                fault_name = "RPDO Timeout";
                break;
            default:
                fault_name = "String interpretation not added";
        }
    }
    if (strstr(fault_name, "String interpretation not added")) {
        draw_text(530, 470, fault_name, 23, OPT_CENTERY);    
    } else {
        draw_text(pos_X, pos_Y, fault_name, 23, OPT_CENTERY);
    }

    //fps 
    //drawFormattedText(20, 450, "wheelSpeed FL: %d", 23, OPT_CENTERY, wheelSpeedFL); testing if this works
    // drawFormattedText(20, 450, "temps: %d, %d, %d, %d, %d, %d, %d, %d\n", 23, OPT_CENTERY, tperiph_br_tire_temps[0],tperiph_br_tire_temps[1],tperiph_br_tire_temps[2],tperiph_br_tire_temps[3],tperiph_br_tire_temps[4],tperiph_br_tire_temps[5],tperiph_br_tire_temps[6],tperiph_br_tire_temps[7]);
    draw_formatted_text(20, 400, "FPS: %d", 23, OPT_CENTERY, FPS);
    end_frame();
}

void DashScreen::draw_main_display(VehicleState& state) {
    // init
    if (!start_frame()) return;
    clear(white.red, white.green, white.blue);
    load_fonts();

    // soc
    if (SOC > 100) {
        SOC = 100;
    }
    draw_progress_bar(Point{100, 25}, 600, 20, SOC, 100, Color{50, 50, 50}, SOC_to_color(SOC), true);

    // acc temp box
    draw_rect(Point{100, 70}, Point{400, 170}, cell_temp_to_color(acc_temp), 32);
    set_main_color(black);
    draw_formatted_text(250, 120, "%d'C", 2, OPT_CENTER, acc_temp);

    // acc pack volt box
    // 	choose color
    Color volt_box_color = green;
    if (acc_volt < PACK_FLASH_VOLT) {
        if (tick % 2 == 0) {
            volt_box_color = red;
        } else {
            volt_box_color = yellow;
        }
    } else if (acc_volt < PACK_WARNING_VOLT) {
        volt_box_color = red;
    } else if (acc_volt < PACK_STD_VOLT) {
        volt_box_color = yellow;
    }

    //	draw box
    draw_rect(Point{400, 70}, Point{700, 170}, volt_box_color, 32);
    draw_formatted_text(550, 120, "%d.%dV", 2, OPT_CENTER, acc_volt / 100, acc_volt % 100);

    // speedometer: convert motor RPM to km/h (gear ratio 11:40, wheel radius 0.190 m)
    float ground_speed = (11.0f / 40.0f) * (2.0f * M_PI * 0.190f) * speed * (60.0f / 1000.0f);
    draw_formatted_text(400, 315, "%d", 1, OPT_CENTER, static_cast<int>(ground_speed));

    // Non-fault bools
    // 	RTD
    if (RTD) draw_rect(Point{100, 200}, Point{300, 250}, green, 32);
    draw_text(200, 225, "RTD", 31, OPT_CENTER);
    // 	fans
    if (fans) draw_rect(Point{100, 250}, Point{300, 300}, green, 32);
    draw_text(200, 275, "FANS", 31, OPT_CENTER);
    // 	precharge
    if (precharged) draw_rect(Point{100, 300}, Point{300, 350}, green, 32);
    draw_text(200, 325, "PRECHGD", 31, OPT_CENTER);

    // Faults
    // 	shutdown circuit
    if (shutdown) {
        Color shutdown_color = yellow;
        if (tick % 2 == 0) {
            shutdown_color = red;
        }
        draw_rect(Point{100, 350}, Point{300, 400}, shutdown_color, 32);
    }
    draw_text(200, 375, "SHDWN", 31, OPT_CENTER);
    // 	motor controller
    if (motor_controller) {
        Color motor_controller_color = yellow;
        if (tick % 2 == 0) {
            motor_controller_color = red;
        }
        draw_rect(Point{100, 400}, Point{300, 450}, motor_controller_color, 32);
    }
    draw_text(200, 425, "MOTR CTRLR", 31, OPT_CENTER);

    // data
    set_main_color(black);
    // LAP
    draw_text(500, 225, "LAP", 31, OPT_CENTERY);
    draw_text(600, 225, lap_time, 31, OPT_CENTERY);
    // GLV
    draw_text(500, 275, "GLV", 31, OPT_CENTERY);
    draw_formatted_text(600, 275, "%d.%dV", 31, OPT_CENTERY, glv / 1000, glv % 1000);
    // DC BUS
    draw_text(500, 325, "DC BUS", 31, OPT_CENTERY);
    draw_formatted_text(675, 325, "%d.%dV", 31, OPT_CENTERY, dc_bus / 10, dc_bus % 10);
    // MOTR TEMP 
    draw_text(500, 375, "MOTR TEMP", 31, OPT_CENTERY);
    draw_formatted_text(650, 375, "%d'C", 31, OPT_CENTERY, motor_temp);
    // CTRLR TEMP
    draw_text(500, 425, "CTRLR TEMP", 31, OPT_CENTERY);
    draw_formatted_text(650, 425, "%d'C", 31, OPT_CENTERY, controller_temp);
    
    //Dials, Testing the Dials
    // drawText(500, 455, "Regen", 23, OPT_CENTERY);
    // drawFormattedText(650, 455, "%d", 23, OPT_CENTERY, regen_state);

    end_frame();
}

void DashScreen::debug_cell_temps(VehicleState& state) {
    // initialize frame
    if (!start_frame()) return;
    clear(white.red, white.green, white.blue);

    // screen label
    set_main_color(black);
    draw_formatted_text(400, 34, "BATTERY CELL TEMPERATURES", 25, OPT_CENTER);

    // cell labels
    set_main_color(black);
    int16_t pos_X = 125;
    for (uint8_t cell = 0; cell < 12; cell++) {
        draw_formatted_text(pos_X, 84, "C%u", CELL_FONT, OPT_CENTER, cell);
        pos_X += CELL_WIDTH / 2;
    }

    // draw grid
    uint16_t pos_Y = 118;
    for (uint8_t segment = 0; segment < 5; segment++) {
        // segment label
        draw_formatted_text(50, pos_Y + (CELL_HEIGHT / 2), "MOD%u", CELL_FONT, OPT_CENTER, segment);
        // cells
        draw_temp_cell(100, pos_Y, segment_temps_A[segment][0]);
        draw_temp_cell(150, pos_Y, segment_temps_A[segment][1]);
        draw_temp_cell(200, pos_Y, segment_temps_A[segment][2]);
        draw_temp_cell(250, pos_Y, segment_temps_A[segment][3]);
        draw_temp_cell(300, pos_Y, segment_temps_A[segment][4]);
        draw_temp_cell(350, pos_Y, segment_temps_A[segment][5]);

        draw_temp_cell(400, pos_Y, segment_temps_B[segment][0]);
        draw_temp_cell(450, pos_Y, segment_temps_B[segment][1]);
        draw_temp_cell(500, pos_Y, segment_temps_B[segment][2]);
        draw_temp_cell(550, pos_Y, segment_temps_B[segment][3]);
        draw_temp_cell(600, pos_Y, segment_temps_B[segment][4]);
        draw_temp_cell(650, pos_Y, segment_temps_B[segment][5]);

        pos_Y += CELL_HEIGHT;
    }

    end_frame();
}

void DashScreen::debug_cell_voltages(VehicleState& state) {
    // initialize frame
    if (!start_frame()) return;
    clear(white.red, white.green, white.blue);

    // screen label
    set_main_color(black);
    draw_formatted_text(400, 34, "BATTERY CELL VOLTAGES", 25, OPT_CENTER);

    // cell labels
    set_main_color(black);
    int16_t pos_X = 150;
    for (uint8_t cell = 0; cell < 6; cell++) {
        draw_formatted_text(pos_X, 84, "CELL%u", CELL_FONT, OPT_CENTER, cell);
        pos_X += CELL_WIDTH;
    }

    // draw grid
    uint16_t pos_Y = 118;
    for (uint8_t segment = 0; segment < 5; segment++) {
        // segment label
        draw_formatted_text(50, pos_Y + (CELL_HEIGHT / 2), "MOD%u", CELL_FONT, OPT_CENTER, segment);
        // cells
        draw_voltage_cell(100, pos_Y, (segment_voltages[segment][0] * 2550 / 255) + 2000);
        draw_voltage_cell(200, pos_Y, (segment_voltages[segment][1] * 2550 / 255) + 2000);
        draw_voltage_cell(300, pos_Y, (segment_voltages[segment][2] * 2550 / 255) + 2000);
        draw_voltage_cell(400, pos_Y, (segment_voltages[segment][3] * 2550 / 255) + 2000);
        draw_voltage_cell(500, pos_Y, (segment_voltages[segment][4] * 2550 / 255) + 2000);
        draw_voltage_cell(600, pos_Y, (segment_voltages[segment][5] * 2550 / 255) + 2000);
        pos_Y += CELL_HEIGHT;
    }

    end_frame();
}

void DashScreen::draw_temp_cell(uint16_t x, uint16_t y, uint8_t temp) {
    uint16_t end_x = x + (CELL_WIDTH / 2), end_y = y + CELL_HEIGHT;
    // draw cell
    draw_rect(Point{x, y}, Point{end_x, end_y}, cell_temp_to_color(temp), 32);
    // draw cell text
    set_main_color(black);
    draw_formatted_text(x + (CELL_WIDTH / 4), y + (CELL_HEIGHT / 2), "%u", CELL_FONT, OPT_CENTER, temp);
}

void DashScreen::draw_voltage_cell(uint16_t x, uint16_t y, uint16_t voltage) {
    uint16_t end_x = x + CELL_WIDTH, end_y = y + CELL_HEIGHT;
    // draw cell
    draw_rect(Point{x, y}, Point{end_x, end_y}, cell_voltage_to_color(voltage), 32);
    // draw cell text
    set_main_color(black);
    draw_formatted_text(x + (CELL_WIDTH / 2), y + (CELL_HEIGHT / 2), "%u.%uV", CELL_FONT, OPT_CENTER, voltage / 1000, voltage % 1000);
}

Color DashScreen::cell_temp_to_color(uint8_t temp) {
    if (temp > CELL_WARNING_TEMP) {
        return red;
    } else if (temp > CELL_NORMAL_TEMP) {
        return yellow;
    } else {
        return green;
    }
}

Color DashScreen::cell_voltage_to_color(uint16_t voltage) {
    if (voltage > CELL_HIGH_VOLT) {
        return blue;
    } else if (voltage > CELL_STD_VOLT) {
        return green;
    } else if (voltage > CELL_LOW_VOLT) {
        return yellow;
    } else {
        return red;
    }
}

Color DashScreen::SOC_to_color(uint8_t SOC) {
    if (SOC > 25) {
        return green;
    } else if (SOC > 10) {
        return yellow;
    } else {
        return red;
    }
}
