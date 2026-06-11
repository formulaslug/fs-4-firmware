//
// Created by Nova Mondal on 5/12/25.
// Edited for FS-4 by Sid Shastri on 2/23/26
#include "dash_screen.hpp"
#include "mbed.h"

DashScreen::DashScreen(PinName mosi, PinName miso, PinName sck, PinName cs, PinName pdn, PinName irq, EvePanel panel)
    : BT817Q(mosi, miso, sck, cs, pdn, irq, panel) {}

static constexpr uint8_t ACC_WARNING_VOLT = 94;
static constexpr uint8_t ACC_WARNING2 = 100;
static constexpr uint8_t ACC_WARNING3 = 103;
static constexpr uint8_t ACC_WARNING_TEMP = 54;

// color intervals
static constexpr uint8_t CELL_WARNING_TEMP = 50;
static constexpr uint8_t CELL_NORMAL_TEMP = 45;
static constexpr uint8_t CELL_LOW_TEMP = 15;

static constexpr uint16_t CELL_HIGH_VOLT = 4000;
static constexpr uint16_t CELL_STD_VOLT = 3400;
static constexpr uint16_t CELL_LOW_VOLT = 2800;

static constexpr uint16_t PACK_STD_VOLT = 10300;
static constexpr uint16_t PACK_WARNING_VOLT = 10000;
static constexpr uint16_t PACK_FLASH_VOLT = 9400;

// debug grid cell dimensions
static constexpr uint16_t CELL_WIDTH = 100;
static constexpr uint16_t CELL_HEIGHT = 68;
static constexpr uint16_t CELL_FONT = 24;

void DashScreen::drawThermalScreen(
    uint8_t acc_temp, uint8_t mtr_temp, uint8_t ctrl_temp,
    float fl_surface, float fl_side,
    float fr_surface, float fr_side,
    float rl_surface, float rl_side,
    float rr_surface, float rr_side,
    float brake_fl, float brake_fr, float brake_rl, float brake_rr,
    float brake_f, float brake_r
) {
    if (failure == startFrame()) return;

    clear(255, 255, 255);
    loadFonts();

    auto setTempColor = [&](int temp, int warnTemp, int redTemp) {
        if (temp >= redTemp) setMainColor(red);
        else if (temp >= warnTemp)
            setMainColor(yellow);
        else
            setMainColor(green);
    };

    // top row
    const int topY = 15;
    const int topH = 70;
    const int topW = 230;
    const int topSpacing = 20;
    int currX = 35;

    auto drawTopBox = [&](const char* label, uint8_t val, int x, int warnTemp, int redTemp) {
        if (val >= redTemp) setMainColor(red);
        else if (val >= warnTemp)
            setMainColor(yellow);
        else
            setMainColor(green);

        drawRect({(uint16_t)x, (uint16_t)topY}, {(uint16_t)(x + topW), (uint16_t)(topY + topH)});
        setMainColor(black);
        drawFormattedText(x + topW / 2, topY + topH / 2, label, 28, OPT_CENTER, val);
    };

    setMainColor((acc_temp > ACC_WARNING_TEMP) ? red : green);
    drawRect({(uint16_t)currX, (uint16_t)topY}, {(uint16_t)(currX + topW), (uint16_t)(topY + topH)});
    setMainColor(black);
    drawFormattedText(currX + topW / 2, topY + topH / 2, "ACC: %dC", 28, OPT_CENTER, acc_temp);
    currX += topW + topSpacing;

    drawTopBox("MTR: %dC", mtr_temp, currX, 80, 100);
    currX += topW + topSpacing;

    drawTopBox("CTRL: %dC", ctrl_temp, currX, 70, 85);

    // main area
    setMainColor(black);
    drawRect({30, 100}, {770, 470}, 3);

    const int boxW = 120;
    const int boxH = 135;
    const int horizontalGap = 140;
    const int verticalGap = 40;
    const int textLineHeight = 35;

    int leftBoxX = 400 - (horizontalGap / 2) - boxW;
    int rightBoxX = 400 + (horizontalGap / 2);

    auto drawWheelPod = [&](int x, int y, float surface, float side, float brk,
                            const char* label, bool isLeft) {
        int avgVal = (int)((surface + side) / 2.0f);
        int iSurface = (int)surface;
        int iSide = (int)side;
        int iBrk = (int)brk;

        setTempColor(avgVal, 80, 100);
        drawRect({(uint16_t)x, (uint16_t)y}, {(uint16_t)(x + boxW), (uint16_t)(y + boxH)});

        // inner boxes
        setMainColor(black);
        drawFormattedText(x + boxW / 2, y + boxH / 2 - 25, label, 30, OPT_CENTER);
        drawFormattedText(x + boxW / 2, y + boxH / 2 + 25, "%dC", 30, OPT_CENTER, avgVal);

        // side text
        int textX = isLeft ? (x - 25) : (x + boxW + 25);
        uint16_t align = isLeft ? OPT_RIGHTX : 0;
        int textStartY = y + 15;

        setTempColor(iSurface, 80, 100);
        drawFormattedText(textX, textStartY, "Surface: %dC", 24, align, iSurface);

        setTempColor(iSide, 80, 100);
        drawFormattedText(textX, textStartY + textLineHeight, "Side: %dC", 24, align, iSide);

        setTempColor(iBrk, 80, 100);
        drawFormattedText(textX, textStartY + (textLineHeight * 2), "Brk Disc: %dC", 24, align, iBrk);
    };

    int frontY = 130;
    int rearY = frontY + boxH + verticalGap;

    drawWheelPod(leftBoxX, frontY, fl_surface, fl_side, brake_fl, "FL:", true);
    drawWheelPod(rightBoxX, frontY, fr_surface, fr_side, brake_fr, "FR:", false);
    drawWheelPod(leftBoxX, rearY, rl_surface, rl_side, brake_rl, "RL:", true);
    drawWheelPod(rightBoxX, rearY, rr_surface, rr_side, brake_rr, "RR:", false);

    // brake balance
    int bb_f = 50;
    int bb_r = 50;

    if ((brake_f + brake_r) > 0.1f) {
        bb_f = (int)(100 * (brake_f / (brake_f + brake_r)));
        bb_r = 100 - bb_f;
    }

    setMainColor(black);
    drawFormattedText(400, 270, "BB:", 30, OPT_CENTER);
    drawFormattedText(400, 300, "%02d/%02d", 30, OPT_CENTER, bb_f, bb_r);

    endFrame();
}
void DashScreen::drawDebugFaultLayout(
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
    int tick
) {

    if (failure == startFrame()) {
        return;
    }
    //-----------\init-------------
    clear(255, 255, 255); // white background for frame why doesnt clear take a color I don't know
    loadFonts();          // Not sure what possible fonts there are to use, above around 17?

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
    setMainColor(black);
    drawText(col1_x, 40, "BAT", 24); // Battery
    drawText(col2_x, 40, "ETC", 24); // Eletronic throttle control
    drawText(col3_x, 40, "SME", 24); // The SME motor controller
    // Seperating lines between columns
    drawLine(Point{col1_x + status_box_offset + 24, 0}, Point{col1_x + status_box_offset + 24, 480}, 10);
    drawLine(Point{col2_x + status_box_offset + 24, 0}, Point{col2_x + status_box_offset + 24, 480}, 10);

    // --- Battery / Accumulator Column ---
    const char* batt_labels[] = {
        "BMS FLT", "IMD FLT", "SHTD", "PCHGD",
        "PCHGING", "PACK V", "GLV", "CELL FLT"
    };
#define GLV_LOW 10800
#define GLV_HIGH 13500
#define MTR_TEMP_HIGH 80
    char batt_values[9][16];
    strcpy(batt_values[0], bms ? "[FAULT]" : "[NO]");
    strcpy(batt_values[1], imd ? "[FAULT]" : "[NO]");
    strcpy(batt_values[2], sdwn ? "[TRUE]" : "[FALSE]");
    strcpy(batt_values[3], pchgd ? "[TRUE]" : "[FALSE]");
    strcpy(batt_values[4], pchgi ? "[TRUE]" : "[FALSE]");
    // strcpy(batt_values[5], chging ? "[TRUE]" : "[FALSE]");
    snprintf(batt_values[5], sizeof(batt_values[5]), "%u.%02u V", packv / 100, packv % 100);
    snprintf(batt_values[6], sizeof(batt_values[6]), "%u mV", glv);
    snprintf(batt_values[7], sizeof(batt_values[7]), "%lu", (unsigned long)cellfault);
    for (uint16_t i = 0; i < 8; ++i) {
        uint16_t y = row_start + i * row_step;
        Color boxColor = green; // default color to green
        if (i == 5) {
            // Flash the box to alarm driver if MC volts below warning point
            if (packv / 100 < ACC_WARNING_VOLT) {
                if (tick % 2 == 0) boxColor = red;
                else
                    boxColor = yellow;
            } else if (packv / 100 < ACC_WARNING2)
                boxColor = red;
            else if (packv / 100 < ACC_WARNING3)
                boxColor = orange;
        } else if (i == 6) {
            // GLV range checking
            if (glv > GLV_LOW && glv < GLV_HIGH) boxColor = green;
            else
                boxColor = red;
        } else if (i == 7) {
            // CELL FLT
            if (cellfault != 0) boxColor = red;
            else
                boxColor = green;
        } else if (strstr(batt_values[i], "FAULT"))
            boxColor = red;
        else if (strstr(batt_values[i], "FALSE"))
            boxColor = mid_gray;
        setMainColor(boxColor);
        drawText(col1_x - 30, y, batt_labels[i], 23, OPT_CENTERY);
        drawText(col1_x + text_offset, y, batt_values[i], 23);
        drawRect(Point{col1_x + status_box_offset, static_cast<unsigned short>(y - font_height_adjustment)}, Point{col1_x + status_box_offset + status_box_size, static_cast<unsigned short>(y + status_box_size - font_height_adjustment)}, boxColor);
        setMainColor(mid_gray);
    }

    // --- ETC Column ---
    const char* etc_labels[] = {
        "RTD", "IMPL", "TS ACTIVE", "PED TRV", "BRK V"
    };
    char etc_values[5][16];
    strcpy(etc_values[0], rtd ? "[TRUE]" : "[FALSE]");
    strcpy(etc_values[1], implausiblity ? "[FAULT]" : "[NO]");
    strcpy(etc_values[2], tsactive ? "[TRUE]" : "[FALSE]");
    snprintf(etc_values[3], sizeof(etc_values[3]), "%u%%", pedaltravel);
    snprintf(etc_values[4], sizeof(etc_values[4]), "%u.%01u V", brakesensev / 10, brakesensev % 10);
    for (uint16_t i = 0; i < 5; ++i) {
        uint16_t y = row_start + i * row_step;
        Color boxColor = green;
        if (i == 3) {
            // Not sure about ranges for pedal travel
            boxColor = mid_gray;
        } else if (i == 4) {
            // Not sure about ranges for brake sensor voltage
            boxColor = mid_gray;
        } else if (strstr(etc_values[i], "FAULT"))
            boxColor = red;
        else if (strstr(etc_values[i], "FALSE"))
            boxColor = mid_gray;
        setMainColor(boxColor);
        drawText(col2_x - 30, y, etc_labels[i], 23, OPT_CENTERY);
        drawText(col2_x + text_offset, y, etc_values[i], 23);
        drawRect(Point{col2_x + status_box_offset, static_cast<unsigned short>(y - font_height_adjustment)}, Point{col2_x + status_box_offset + status_box_size, static_cast<unsigned short>(y + status_box_size - font_height_adjustment)}, boxColor);
        setMainColor(mid_gray);
    }

    // --- SME Column ---
    const char* sme_labels[] = {
        "TMP OVR", "RUNNING", "PWR ON", "PWR RDY",
        "MTR TMP", "FLT CODE", "FLT LEVEL"
    };
    char sme_values[7][16];
    strcpy(sme_values[0], ctrlovertemp ? "[TRUE]" : "[FALSE]");
    strcpy(sme_values[1], running ? "[TRUE]" : "[FALSE]");
    strcpy(sme_values[2], poweron ? "[TRUE]" : "[FALSE]");
    strcpy(sme_values[3], powerrdy ? "[TRUE]" : "[FALSE]");
    snprintf(sme_values[4], sizeof(sme_values[4]), "%u.%01u 'C", motortemp / 10, motortemp % 10);
    snprintf(sme_values[5], sizeof(sme_values[5]), "%u", faultcode);
    snprintf(sme_values[6], sizeof(sme_values[6]), "%u", faultlevel);
    for (uint16_t i = 0; i < 7; ++i) {
        uint16_t y = row_start + i * row_step;
        Color boxColor = green;
        if (i == 4) {
            // motor temp
            if (motortemp < MTR_TEMP_HIGH) boxColor = green;
            else
                boxColor = red;
        } else if (i == 5) {
            // fault code
            if (faultcode <= 0) boxColor = green;
            else
                boxColor = red;
        } else if (i == 6) {
            // fault level
            if (faultlevel != 0) boxColor = red;
            else
                boxColor = green;
        } else if (strstr(sme_values[i], "FAULT"))
            boxColor = red;
        else if (strstr(sme_values[i], "FALSE"))
            boxColor = mid_gray;
        setMainColor(boxColor);
        drawText(col3_x - 30, y, sme_labels[i], 23, OPT_CENTERY);
        drawText(col3_x + text_offset, y, sme_values[i], 23);
        drawRect(Point{col3_x + status_box_offset, static_cast<unsigned short>(y - font_height_adjustment)}, Point{col3_x + status_box_offset + status_box_size, static_cast<unsigned short>(y + status_box_size - font_height_adjustment)}, boxColor);
        setMainColor(mid_gray);
    }

    // String interpretations of common fault codes
    const uint16_t xPos = 530;
    const uint16_t yPos = 430;
    drawText(xPos, 400, "MTR CTL: ", 23, OPT_CENTERY);
    if (faultcode <= 0) drawText(xPos, yPos, "No faults", 23, OPT_CENTERY);
    else if (faultcode == 1)
        drawText(xPos, yPos, "Over Voltage", 23, OPT_CENTERY);
    else if (faultcode == 2)
        drawText(xPos, yPos, "Under Voltage", 23, OPT_CENTERY);
    else if (faultcode == 3)
        drawText(xPos, yPos, "User Over Voltage", 23, OPT_CENTERY);
    else if (faultcode == 26)
        drawText(xPos, yPos, "High Voltage", 23, OPT_CENTERY);
    else if (faultcode == 25)
        drawText(xPos, yPos, "Low Voltage", 23, OPT_CENTERY);
    else if (faultcode == 62)
        drawText(xPos, yPos, "Heartbeat Timeout", 23, OPT_CENTERY);
    else if (faultcode == 63)
        drawText(xPos, yPos, "RPDO Timeout", 23, OPT_CENTERY);
    else
        drawText(530, 470, "String interpretation not added", 23, OPT_CENTERY);
    endFrame();

    //fps 
    drawFormattedText(20, 400, "fps: %d", 23, OPT_CENTERY, fps);
     

}

void DashScreen::drawMainDisplay(bool shtd, bool mtr_ctrl, bool rtd, bool pchg, bool fans, uint16_t acc_volt, uint8_t acc_temp, uint8_t soc, int tick, uint16_t speed, const char* lap_time, uint16_t glv, uint8_t mtr_temp, uint8_t ctrl_temp, uint16_t dc_bus, uint8_t regen_state) {
    // init
    if (!startFrame()) return;
    clear(white.red, white.green, white.blue);
    loadFonts();

    // soc
    if (soc > 100) soc = 100;
    drawProgressBar(Point{100, 25}, 600, 20, soc, 100, Color{50, 50, 50}, socToColor(soc), true);

    // acc temp box
    drawRect(Point{100, 70}, Point{400, 170}, cellTempToColor(acc_temp), 32);
    setMainColor(black);
    drawFormattedText(250, 120, "%d'C", 2, OPT_CENTER, acc_temp);

    // acc pack volt box
    // 	choose color
    Color volt_box_color = green;
    if (acc_volt < PACK_FLASH_VOLT) {
        if (tick % 2 == 0) volt_box_color = red;
        else
            volt_box_color = yellow;
    } else if (acc_volt < PACK_WARNING_VOLT)
        volt_box_color = red;
    else if (acc_volt < PACK_STD_VOLT)
        volt_box_color = yellow;
    //	draw box
    drawRect(Point{400, 70}, Point{700, 170}, volt_box_color, 32);
    drawFormattedText(550, 120, "%d.%dV", 2, OPT_CENTER, acc_volt / 100, acc_volt % 100);

    // speedometer: convert motor RPM to km/h (gear ratio 11:40, wheel radius 0.190 m)
    float ground_speed = (11.0f / 40.0f) * (2.0f * M_PI * 0.190f) * speed * (60.0f / 1000.0f);
    drawFormattedText(400, 315, "%d", 1, OPT_CENTER, static_cast<int>(ground_speed));

    // Non-fault bools
    // 	RTD
    if (rtd) drawRect(Point{100, 200}, Point{300, 250}, green, 32);
    drawText(200, 225, "RTD", 31, OPT_CENTER);
    // 	fans
    if (fans) drawRect(Point{100, 250}, Point{300, 300}, green, 32);
    drawText(200, 275, "FANS", 31, OPT_CENTER);
    // 	precharge
    if (pchg) drawRect(Point{100, 300}, Point{300, 350}, green, 32);
    drawText(200, 325, "PCHG", 31, OPT_CENTER);

    // Faults
    // 	shutdown circuit
    if (shtd) {
        Color shtd_c = yellow;
        if (tick % 2 == 0) shtd_c = red;
        drawRect(Point{100, 350}, Point{300, 400}, shtd_c, 32);
    }
    drawText(200, 375, "SHTD", 31, OPT_CENTER);
    // 	motor controller
    if (mtr_ctrl) {
        Color mctrl_c = yellow;
        if (tick % 2 == 0) mctrl_c = red;
        drawRect(Point{100, 400}, Point{300, 450}, mctrl_c, 32);
    }
    drawText(200, 425, "MCTRL", 31, OPT_CENTER);

    // data
    setMainColor(black);
    // LAP
    drawText(500, 225, "LAP", 31, OPT_CENTERY);
    drawText(600, 225, lap_time, 31, OPT_CENTERY);
    // GLV
    drawText(500, 275, "GLV", 31, OPT_CENTERY);
    drawFormattedText(600, 275, "%d.%dV", 31, OPT_CENTERY, glv / 1000, glv % 1000);
    // DC BUS
    drawText(500, 325, "DC BUS", 31, OPT_CENTERY);
    drawFormattedText(675, 325, "%d.%dV", 31, OPT_CENTERY, dc_bus / 10, dc_bus % 10);
    // MTR
    drawText(500, 375, "MTR", 31, OPT_CENTERY);
    drawFormattedText(650, 375, "%d'C", 31, OPT_CENTERY, mtr_temp);
    // CTRL
    drawText(500, 425, "CTRL", 31, OPT_CENTERY);
    drawFormattedText(650, 425, "%d'C", 31, OPT_CENTERY, ctrl_temp);
    
    //Dials, Testing the Dials
    // drawText(500, 455, "Regen", 23, OPT_CENTERY);
    // drawFormattedText(650, 455, "%d", 23, OPT_CENTERY, regen_state);

    endFrame();
}

void DashScreen::debugCellTemps(const uint8_t seg_temps_A[5][6], const uint8_t seg_temps_B[5][6]) {
    // initialize frame
    if (!startFrame()) return;
    clear(white.red, white.green, white.blue);

    // screen label
    setMainColor(black);
    drawFormattedText(400, 34, "BATTERY CELL TEMPERATURES", 25, OPT_CENTER);

    // cell labels
    setMainColor(black);
    int16_t x = 125;
    for (uint8_t cell = 0; cell < 12; cell++) {
        drawFormattedText(x, 84, "C%u", CELL_FONT, OPT_CENTER, cell);
        x += CELL_WIDTH / 2;
    }

    // draw grid
    uint16_t y = 118;
    for (uint8_t seg = 0; seg < 5; seg++) {
        // segment label
        drawFormattedText(50, y + (CELL_HEIGHT / 2), "MOD%u", CELL_FONT, OPT_CENTER, seg);
        // cells
        drawTempCell(100, y, seg_temps_A[seg][0]);
        drawTempCell(150, y, seg_temps_A[seg][1]);
        drawTempCell(200, y, seg_temps_A[seg][2]);
        drawTempCell(250, y, seg_temps_A[seg][3]);
        drawTempCell(300, y, seg_temps_A[seg][4]);
        drawTempCell(350, y, seg_temps_A[seg][5]);

        drawTempCell(400, y, seg_temps_B[seg][0]);
        drawTempCell(450, y, seg_temps_B[seg][1]);
        drawTempCell(500, y, seg_temps_B[seg][2]);
        drawTempCell(550, y, seg_temps_B[seg][3]);
        drawTempCell(600, y, seg_temps_B[seg][4]);
        drawTempCell(650, y, seg_temps_B[seg][5]);

        y += CELL_HEIGHT;
    }

    endFrame();
}

void DashScreen::debugCellVolts(const uint8_t seg_volts[5][6]) {
    // initialize frame
    if (!startFrame()) return;
    clear(white.red, white.green, white.blue);

    // screen label
    setMainColor(black);
    drawFormattedText(400, 34, "BATTERY CELL VOLTAGES", 25, OPT_CENTER);

    // cell labels
    setMainColor(black);
    int16_t x = 150;
    for (uint8_t cell = 0; cell < 6; cell++) {
        drawFormattedText(x, 84, "CELL%u", CELL_FONT, OPT_CENTER, cell);
        x += CELL_WIDTH;
    }

    // draw grid
    uint16_t y = 118;
    for (uint8_t seg = 0; seg < 5; seg++) {
        // segment label
        drawFormattedText(50, y + (CELL_HEIGHT / 2), "MOD%u", CELL_FONT, OPT_CENTER, seg);
        // cells
        drawVoltCell(100, y, (seg_volts[seg][0] * 2550 / 255) + 2000);
        drawVoltCell(200, y, (seg_volts[seg][1] * 2550 / 255) + 2000);
        drawVoltCell(300, y, (seg_volts[seg][2] * 2550 / 255) + 2000);
        drawVoltCell(400, y, (seg_volts[seg][3] * 2550 / 255) + 2000);
        drawVoltCell(500, y, (seg_volts[seg][4] * 2550 / 255) + 2000);
        drawVoltCell(600, y, (seg_volts[seg][5] * 2550 / 255) + 2000);
        y += CELL_HEIGHT;
    }

    endFrame();
}

void DashScreen::drawTempCell(uint16_t x, uint16_t y, uint8_t temp) {
    uint16_t br_x = x + (CELL_WIDTH / 2), br_y = y + CELL_HEIGHT;
    // draw cell
    drawRect(Point{x, y}, Point{br_x, br_y}, cellTempToColor(temp), 32);
    // draw cell text
    setMainColor(black);
    drawFormattedText(x + (CELL_WIDTH / 4), y + (CELL_HEIGHT / 2), "%u", CELL_FONT, OPT_CENTER, temp);
}

void DashScreen::drawVoltCell(uint16_t x, uint16_t y, uint16_t volt) {
    uint16_t br_x = x + CELL_WIDTH, br_y = y + CELL_HEIGHT;
    // draw cell
    drawRect(Point{x, y}, Point{br_x, br_y}, cellVoltToColor(volt), 32);
    // draw cell text
    setMainColor(black);
    drawFormattedText(x + (CELL_WIDTH / 2), y + (CELL_HEIGHT / 2), "%u.%uV", CELL_FONT, OPT_CENTER, volt / 1000, volt % 1000);
}

Color DashScreen::cellTempToColor(uint8_t temp) {
    if (temp > CELL_WARNING_TEMP) return red;
    else if (temp > CELL_NORMAL_TEMP)
        return yellow;
    else return green;
}

Color DashScreen::cellVoltToColor(uint16_t volt) {
    if (volt > CELL_HIGH_VOLT) return blue;
    else if (volt > CELL_STD_VOLT)
        return green;
    else if (volt > CELL_LOW_VOLT)
        return yellow;
    else
        return red;
}

Color DashScreen::socToColor(uint8_t soc) {
    if (soc > 25) return green;
    else if (soc > 10)
        return yellow;
    else
        return red;
}
