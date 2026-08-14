//#include "debounced_digital_in.h"
#include "BT817Q.hpp"
// CANbus.h / dbcc not used: CAN signals decoded manually from msg.data[]
#include "dash_screen.hpp"
#include "mbed.h"
#include "pin_config.hpp"
#include "vehicle_state.hpp"

// TODO: I had to put this here to get it to compile, but I have no
// idea what the actual clock setup on steering wheel board is supposed to be
extern "C" void SetSysClock(void) {}

DashScreen screen{
    PIN_MOSI, PIN_MISO, PIN_SCK, PIN_CS, PIN_PDN, PIN_IRQ, EvePresets::CFAF800480H0
};
CAN can{PIN_CAN_RX, PIN_CAN_TX, CAN_FREQUENCY};
CANMessage msg;
DigitalIn layoutBtn{PIN_LAYOUT};
bool lastLayoutBtnState;
//DebouncedDigitalIn debouncedLayoutBtn{layoutBtn, 5};
uint8_t currentScreen = 0;
EventQueue queue = EventQueue{EVENTS_EVENT_SIZE * 32};
AnalogIn driveDial{PIN_MODE};
AnalogIn tractionDial{PIN_TC};
AnalogIn regenDial{PIN_REGEN};
uint8_t driveState;
uint8_t tractionState;
uint8_t regenState;
bool pedalCalActive = false;
uint8_t pedalCalStep = 0;

// g_dbcc_state removed — all CAN signals decoded manually from msg.data[]
VehicleState vsm_state;
static int tick = 0;
uint32_t lastFrame = 0;
uint32_t curFrame = 0;
uint16_t fps = 0;

int main() {
    printf("Steering Wheel!!\n");
    screen.init(EvePresets::CFAF800480H0);
    screen.setRotate(1); //Inverted Landscape since dash is mounted upsidedown
    // debouncedLayoutBtn.set_valid_read_count(7);
    can.attach(&canISR, CAN::RxIrq); // callback function to handle CAN interrupts
    queue.call_every(10ms, &readLayoutButton);
    queue.call_every(10ms, &readPedalCalButton);
    queue.call_every(10ms, &readDials);
    queue.call_every(10ms, &drawScreenLayout); //100hz
    queue.call_every(100ms, []() { tick++; }); //Lambda expression to increase ticks every 100ms, or I hope it runs every 100ms
    queue.call_every(1s, &readFPS);
    queue.dispatch_forever();
    return 0;
}

/**
 * @brief Function to read button state and cycles through possible layouts on button press
 * These are the what values correspond to what screen
 */
void readLayoutButton() {
    // const bool debounced_button_state = debouncedLayoutBtn.read();
    const bool debounced_button_state = layoutBtn.read();
    if (debounced_button_state && !lastLayoutBtnState) {
        currentScreen = (currentScreen >= 4) ? 0 : (currentScreen + 1);
    }
    lastLayoutBtnState = debounced_button_state;
}

/**
 * @brief Function to read button state for pedal calibration screen
 */
void readPedalCalButton() {
    static uint16_t heldTicks = 0;
    static bool lastState = false;
    const bool state = layoutBtn.read();
    if (state) {
        heldTicks++;
    } else {
        if (lastState && heldTicks >= 300) {
            pedalCalActive = !pedalCalActive;
            pedalCalStep = 0;
        } else if (lastState && pedalCalActive) {
            uint8_t data[1] = {pedalCalStep};
            CANMessage msg{0x4B0, data, 1};
            can.write(msg);
            pedalCalStep++;
            if (pedalCalStep > 3) pedalCalActive = false;
        }
        heldTicks = 0;
    }
    lastState = state;
}

/**
 * @brief Function to read dial state
 *  3.3V, 2.2V, 1.1V, GND
 *  Not sure how jiterry these readings
 *  I can compare floats or just cast it to an int
 */
void readDials() {
    // 0 - 3.3V
    // 0 - 4096, each volt is 1241
    // ADC is 12 Bits ADC12_IN1
    uint16_t rawDriveVal = driveDial.read_u16() >> 4;
    if (rawDriveVal >= 3351) driveState = 3; // > 2.7V
    else if (rawDriveVal >= 2110)
        driveState = 2; // > 1.7V
    else if (rawDriveVal >= 744)
        driveState = 1; // > 0.6V
    else
        driveState = 0; // ground, idk what to do here

    uint16_t rawTractionVal = tractionDial.read_u16() >> 4;
    if (rawTractionVal >= 3351) tractionState = 3; // > 2.7V
    else if (rawTractionVal >= 2110)
        tractionState = 2; // > 1.7V
    else if (rawTractionVal >= 744)
        tractionState = 1; // > 0.6V
    else
        tractionState = 0; // ground

    uint16_t rawRegenVal = regenDial.read_u16() >> 4;
    if (rawRegenVal >= 3351) regenState = 3; // > 2.7V
    else if (rawRegenVal >= 2110)
        regenState = 2; // > 1.7V
    else if (rawRegenVal >= 744)
        regenState = 1; // > 0.6V
    else
        regenState = 0; // ground

    // between above 3, above 2 <3,

    // Send the CAN Message
    // printf("Drive State: %d\n", driveState);
    // printf("Traction State: %d\n", tractionState);
    // printf("Regen State: %d\n", regenState);
    sendCANMessage();
}

/**
 * @brief Function to draw screen layout
 * Layouts depend on currentScreen
 * 0: main
 * 1: Thermal
 * 2: CellTemps
 * 3: CellVolts
 * 4: Fault
 */
void drawScreenLayout() {
    if (pedalCalActive) {
        screen.drawPedalCalScreen(
            vsm_state.vcu_etc_apps1_mv, vsm_state.vcu_etc_apps2_mv, vsm_state.vcu_etc_bpps_mv, pedalCalStep
        );
        return;
    }
    // Render the screen like how it was done before?
    // Should look into optimal ways of doing this
    // Also add fps overlay to the screens
    // Also might need to edit screen layouts since stuff seems to have been changed

    switch (currentScreen) {
    case 0:
        screen.drawMainDisplay(
            vsm_state.batt_status_shutdown_final,
            vsm_state.sme_temp_faultlevel > 0 && vsm_state.sme_temp_faultlevel < 4,
            vsm_state.vcu_etc_ready_to_drive,
            vsm_state.vcu_etc_batt_precharge_done,
            true /*fans*/,
            vsm_state.batt_power_pack_voltage,
            vsm_state.batt_stat_high_cell_temp,
            vsm_state.batt_power_soc,
            tick,
            vsm_state.sme_trqspd_speed,
            "lap", // laptime
            vsm_state.batt_status_glv_voltage,
            vsm_state.sme_temp_motortemperature,
            vsm_state.sme_temp_controllertemperature,
            vsm_state.sme_temp_dc_bus_v,
            regenState
        );
        break;
    case 1: {
        //Lambda expression to take the average temps
        auto avgTireTemps = [](const uint8_t temps[8]) -> float {
            uint16_t sum = 0;
            for (int i = 0; i < 8; i++)
                sum += temps[i];
            return sum / 8.0f;
        };
        screen.drawThermalScreen(
            vsm_state.batt_stat_high_cell_temp,
            vsm_state.sme_temp_motortemperature,
            vsm_state.sme_temp_controllertemperature,
            avgTireTemps(vsm_state.tperiph_fl_tire_temps), (float)vsm_state.tperiph_fl_side_temp,
            avgTireTemps(vsm_state.tperiph_fr_tire_temps), (float)vsm_state.tperiph_fr_side_temp,
            avgTireTemps(vsm_state.tperiph_bl_tire_temps), (float)vsm_state.tperiph_bl_side_temp,
            avgTireTemps(vsm_state.tperiph_br_tire_temps), (float)vsm_state.tperiph_br_side_temp,
            vsm_state.izze_brake_fl, vsm_state.izze_brake_fr,
            vsm_state.izze_brake_rl, vsm_state.izze_brake_rr,
            (float)vsm_state.vcu_etc_brake_pressure_front,
            (float)vsm_state.vcu_etc_brake_pressure_rear
        );
        
        break;
    }
    case 2:
        screen.debugCellTemps(vsm_state.batt_mod_temps_A, vsm_state.batt_mod_temps_B);
        break;
    case 3:
        screen.debugCellVolts(vsm_state.batt_mod_volts);
        break;
    case 4:
        printf("In Faults Display \n");
        screen.drawDebugFaultLayout(
            vsm_state.batt_status_bms_fault,
            vsm_state.batt_status_imd_fault,
            vsm_state.batt_status_shutdown_out,
            vsm_state.batt_status_precharge_done,
            vsm_state.batt_status_precharging,
            vsm_state.batt_status_charging,
            vsm_state.batt_power_pack_voltage,
            vsm_state.batt_status_glv_voltage,
            vsm_state.batt_status_fault_index,
            vsm_state.vcu_etc_ready_to_drive,
            (vsm_state.vcu_etc_implausibility_apps_out_of_range | vsm_state.vcu_etc_implausibility_bpps_out_of_range | vsm_state.vcu_etc_implausibility_apps_deviation | vsm_state.vcu_etc_implausibility_bse_out_of_range | vsm_state.vcu_etc_implausibility_brake_and_accel), // lot of implausbility options now
            vsm_state.vcu_etc_batt_precharge_done,
            vsm_state.vcu_etc_brake_pedal_travel,
            vsm_state.vcu_etc_bpps_he,
            vsm_state.sme_trqspd_controller_overtermp,
            vsm_state.sme_trqspd_running,
            vsm_state.sme_trqspd_powering_enabled,
            vsm_state.sme_trqspd_powering_ready,
            vsm_state.sme_temp_motortemperature,
            vsm_state.sme_temp_faultcode,
            vsm_state.sme_temp_faultlevel,
            fps,
            //vsm_state.tperiph_br_tire_temps,
            tick
        );
        break;
    }
}
/**
 * @brief Called when there is an incomming CAN message from interrupt
 *  Just adds the decoding process to the queue
 */
void canISR() {
    queue.call(processCANMessage);
}
/**
 * @brief Decoding the CAN message receieved
 * Worried about if the eventQueue is busy with other tasks and the original message is missed
 * Not sure how long the other tasks take and the maximum time between messages
 * Entirely generated by a python script although I had to manually uncomment the data that was used
 * I originally thought CAN dbcc provided automatic scaling, it provides the automatic packaging and unpacking of CAN Messages 
 *
 */
void processCANMessage() {
    while (can.read(msg)) {
        switch (msg.id) {
        case 0x192:   // VCU_TPDO_PEDAL_TRAVEL (BPPS_HE, brake pedal travel)
        case 0x193:   // VCU_TPDO_STATUS (RTD, precharge, brake pressures, implausibilities)
        case 0x482:   // SME_TPDO_Torque_speed
        case 0x682:   // SME_TPDO_Temperature
        case 0x391:   // BATT_TPDO_STATUS
        case 0x392:   // BATT_TPDO_POWER
        case 0x393:   // BATT_TPDO_CELL_STATS
        case 0xa0001: // VDM_GPS_DATA
        case 0x1a5:   // TPERIPH_FL_TPDO_DATA
        case 0x1a6:   // TPERIPH_FR_TPDO_DATA
        case 0x1a7:   // TPERIPH_BL_TPDO_DATA
        case 0x1a8:   // TPERIPH_BR_TPDO_DATA
        case 0x2a5:   // TPERIPH_FL_TPDO_TIRETEMP
        case 0x2a6:   // TPERIPH_FR_TPDO_TIRETEMP
        case 0x2a7:   // TPERIPH_BL_TPDO_TIRETEMP
        case 0x2a8:   // TPERIPH_BR_TPDO_TIRETEMP
        case 0x494:   // BATT_TPDO_MOD0_VOLTS
        case 0x497:   // BATT_TPDO_MOD1_VOLTS
        case 0x49a:   // BATT_TPDO_MOD2_VOLTS
        case 0x49d:   // BATT_TPDO_MOD3_VOLTS
        case 0x4a0:   // BATT_TPDO_MOD4_VOLTS
        case 0x495:   // BATT_TPDO_MOD0_TEMPS_A
        case 0x496:   // BATT_TPDO_MOD0_TEMPS_B (cells 6-11, unused by display)
        case 0x498:   // BATT_TPDO_MOD1_TEMPS_A
        case 0x499:   // BATT_TPDO_MOD1_TEMPS_B
        case 0x49b:   // BATT_TPDO_MOD2_TEMPS_A
        case 0x49c:   // BATT_TPDO_MOD2_TEMPS_B
        case 0x49e:   // BATT_TPDO_MOD3_TEMPS_A
        case 0x49f:   // BATT_TPDO_MOD3_TEMPS_B
        case 0x4a1:   // BATT_TPDO_MOD4_TEMPS_A
        case 0x4a2:   // BATT_TPDO_MOD4_TEMPS_B
        case 0x4c4:   // IZZE_BRAKETEMP_S1M1 (CH1-4)
        case 0x4c5:   // IZZE_BRAKETEMP_S1M2 (CH5-8)
        case 0x4c6:   // IZZE_BRAKETEMP_S1M3 (CH9-12)
        case 0x4c7:   // IZZE_BRAKETEMP_S1M4 (CH13-16)
        {
            //printf("reading from CAN\n");
            // Direct msg.data[] extraction — no dbcc/CANbus overhead.
            // All signals are little-endian (Intel byte order) unless noted as Motorola.
            if (false) {
            } else if (msg.id == 0x192) { // VCU_TPDO_PEDAL_TRAVEL
                // BPPS_HE: bits 32-47 (bytes 4-5), low byte sufficient for uint8_t display
                vsm_state.vcu_etc_bpps_he = msg.data[4];
                // BRAKE_PEDAL_TRAVEL: bit 56 (byte 7), raw = 0-100 %
                vsm_state.vcu_etc_brake_pedal_travel = msg.data[7];
                vsm_state.vcu_etc_apps1_mv = (uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8);
                vsm_state.vcu_etc_apps2_mv = (uint16_t)msg.data[2] | ((uint16_t)msg.data[3] << 8);
                vsm_state.vcu_etc_bpps_mv  = (uint16_t)msg.data[4] | ((uint16_t)msg.data[5] << 8);
            } else if (msg.id == 0x193) { // VCU_TPDO_STATUS
                // BRAKE_PRESSURE_FRONT: bits 16-31 (bytes 2-3), little-endian
                vsm_state.vcu_etc_brake_pressure_front = (uint16_t)msg.data[2] | ((uint16_t)msg.data[3] << 8);
                // BRAKE_PRESSURE_REAR: bits 32-47 (bytes 4-5), little-endian
                vsm_state.vcu_etc_brake_pressure_rear  = (uint16_t)msg.data[4] | ((uint16_t)msg.data[5] << 8);
                // Status flags packed in bytes 0-1
                vsm_state.vcu_etc_ready_to_drive                   = (msg.data[0] >> 0) & 0x01;
                vsm_state.vcu_etc_batt_precharge_done              = (msg.data[0] >> 3) & 0x01;
                vsm_state.vcu_etc_implausibility_apps_out_of_range = (msg.data[0] >> 4) & 0x01;
                vsm_state.vcu_etc_implausibility_bpps_out_of_range = (msg.data[0] >> 5) & 0x01;
                vsm_state.vcu_etc_implausibility_apps_deviation    = (msg.data[0] >> 6) & 0x01;
                vsm_state.vcu_etc_implausibility_bse_out_of_range  = (msg.data[0] >> 7) & 0x01;
                vsm_state.vcu_etc_implausibility_brake_and_accel   = (msg.data[1] >> 0) & 0x01;
            } else if (msg.id == 0x482) { // SME_TPDO_Torque_speed
                // Speed: bits 0-15 (bytes 0-1), little-endian, raw RPM
                vsm_state.sme_trqspd_speed               = (uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8);
                vsm_state.sme_trqspd_controller_overtermp = (msg.data[4] >> 6) & 0x01; // bit 38
                vsm_state.sme_trqspd_running               = (msg.data[5] >> 1) & 0x01; // bit 41
                vsm_state.sme_trqspd_powering_enabled      = (msg.data[5] >> 4) & 0x01; // bit 44
                vsm_state.sme_trqspd_powering_ready        = (msg.data[5] >> 5) & 0x01; // bit 45
            } else if (msg.id == 0x682) { // SME_TPDO_Temperature
                // Motor/ctrl temps: raw byte has -40 offset in DBC; store physical °C, clamped to 0
                vsm_state.sme_temp_motortemperature      = (msg.data[0] >= 40) ? msg.data[0] - 40 : 0;
                vsm_state.sme_temp_controllertemperature = (msg.data[1] >= 40) ? msg.data[1] - 40 : 0;
                // DC Bus V: bits 16-31 (bytes 2-3), little-endian, raw = V*10; display divides by 10
                vsm_state.sme_temp_dc_bus_v  = (uint16_t)msg.data[2] | ((uint16_t)msg.data[3] << 8);
                vsm_state.sme_temp_faultcode  = msg.data[4]; // bits 32-39
                vsm_state.sme_temp_faultlevel = msg.data[5]; // bits 40-47
            } else if (msg.id == 0x391) { // BATT_TPDO_STATUS
                // Status flags in byte 0
                vsm_state.batt_status_bms_fault      = (msg.data[0] >> 0) & 0x01;
                vsm_state.batt_status_imd_fault       = (msg.data[0] >> 1) & 0x01;
                vsm_state.batt_status_shutdown_final  = (msg.data[0] >> 2) & 0x01;
                vsm_state.batt_status_shutdown_out    = (msg.data[0] >> 4) & 0x01;
                vsm_state.batt_status_precharging     = (msg.data[0] >> 5) & 0x01;
                vsm_state.batt_status_precharge_done  = (msg.data[0] >> 6) & 0x01;
                vsm_state.batt_status_charging        = (msg.data[0] >> 7) & 0x01;
                // Fault index: byte 4 (bits 32-39)
                vsm_state.batt_status_fault_index = msg.data[4];
                // GLV voltage: bits 40-55 (bytes 5-6), little-endian, raw = mV (display * 0.001 = V)
                vsm_state.batt_status_glv_voltage = (uint16_t)msg.data[5] | ((uint16_t)msg.data[6] << 8);
            } else if (msg.id == 0x392) { // BATT_TPDO_POWER
                // Pack voltage: bits 0-15 (bytes 0-1), little-endian, raw = cV (display * 0.01 = V)
                vsm_state.batt_power_pack_voltage = (uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8);
                // SOC: bits 50-59, 10-bit little-endian spanning bytes 6-7
                {
                    uint16_t soc_raw = (uint16_t)(msg.data[6] >> 2) | ((uint16_t)(msg.data[7] & 0x0f) << 6);
                    vsm_state.batt_power_soc = (uint8_t)(soc_raw / 10); // scaling 0.1 → raw/10 = integer %
                }
            } else if (msg.id == 0x393) { // BATT_TPDO_CELL_STATS
                // High cell temp: byte 1 (bits 8-15), scaling 0.25 → raw/4 = °C
                vsm_state.batt_stat_high_cell_temp = msg.data[1] / 4;
            } else if (msg.id == 0xa0001) { // VDM_GPS_DATA — Motorola/big-endian
                // GPS speed: Motorola start-bit 7, 16-bit big-endian at bytes 0-1
                // raw = speed * 100; display * 0.01 = km/h
                vsm_state.vdm_gps_speed = ((uint16_t)msg.data[0] << 8) | msg.data[1];
            }
            // Tire peripheral side temps + wheelspeed (little-endian)
            else if (msg.id == 0x1a5) {
                vsm_state.tperiph_fl_side_temp = msg.data[6]; // bits 48-55 (byte 6)
                { int16_t ws = (int16_t)((uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8));
                  vsm_state.tperiph_fl_data_wheelspeed = (ws >= 0) ? (uint8_t)(ws / 10) : 0; }
            } else if (msg.id == 0x1a6) {
                vsm_state.tperiph_fr_side_temp = msg.data[6];
                { int16_t ws = (int16_t)((uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8));
                  vsm_state.tperiph_fr_data_wheelspeed = (ws >= 0) ? (uint8_t)(ws / 10) : 0; }
            } else if (msg.id == 0x1a7) {
                vsm_state.tperiph_bl_side_temp = msg.data[6];
                { int16_t ws = (int16_t)((uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8));
                  vsm_state.tperiph_bl_data_wheelspeed = (ws >= 0) ? (uint8_t)(ws / 10) : 0; }
            } else if (msg.id == 0x1a8) {
                vsm_state.tperiph_br_side_temp = msg.data[6];
                { int16_t ws = (int16_t)((uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8));
                  vsm_state.tperiph_br_data_wheelspeed = (ws >= 0) ? (uint8_t)(ws / 10) : 0; }
            }
            // Tire surface temps: byte N = sensor N+1, raw °C direct
            else if (msg.id == 0x2a5) {
                for (int i = 0; i < 8 && i < msg.len; i++) vsm_state.tperiph_fl_tire_temps[i] = msg.data[i];
            } else if (msg.id == 0x2a6) {
                for (int i = 0; i < 8 && i < msg.len; i++) vsm_state.tperiph_fr_tire_temps[i] = msg.data[i];
            } else if (msg.id == 0x2a7) {
                for (int i = 0; i < 8 && i < msg.len; i++) vsm_state.tperiph_bl_tire_temps[i] = msg.data[i];
            } else if (msg.id == 0x2a8) {
                for (int i = 0; i < 8 && i < msg.len; i++) vsm_state.tperiph_br_tire_temps[i] = msg.data[i];
            }
            // Battery cell voltages: byte N = CELL_N raw (display: raw * 0.01 + 2.0 = V)
            else if (msg.id == 0x494) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_volts[0][i] = msg.data[i];
            } else if (msg.id == 0x497) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_volts[1][i] = msg.data[i];
            } else if (msg.id == 0x49a) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_volts[2][i] = msg.data[i];
            } else if (msg.id == 0x49d) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_volts[3][i] = msg.data[i];
            } else if (msg.id == 0x4a0) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_volts[4][i] = msg.data[i];
            }
            // Battery cell temps A: byte N = CELL_N raw; scaling 0.25 → raw/4 = °C
            else if (msg.id == 0x495) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_temps_A[0][i] = msg.data[i] / 4;
            } else if (msg.id == 0x498) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_temps_A[1][i] = msg.data[i] / 4;
            } else if (msg.id == 0x49b) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_temps_A[2][i] = msg.data[i] / 4;
            } else if (msg.id == 0x49e) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_temps_A[3][i] = msg.data[i] / 4;
            } else if (msg.id == 0x4a1) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_temps_A[4][i] = msg.data[i] / 4;
            }
            // Battery cell temps B (cells 6-11): byte N = CELL_(N+6) raw; scaling 0.25 → raw/4 = °C
            else if (msg.id == 0x496) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_temps_B[0][i] = msg.data[i] / 4;
            } else if (msg.id == 0x499) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_temps_B[1][i] = msg.data[i] / 4;
            } else if (msg.id == 0x49c) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_temps_B[2][i] = msg.data[i] / 4;
            } else if (msg.id == 0x49f) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_temps_B[3][i] = msg.data[i] / 4;
            } else if (msg.id == 0x4a2) {
                for (int i = 0; i < 6; i++) vsm_state.batt_mod_temps_B[4][i] = msg.data[i] / 4;
            }
            // IZZE brake disc temps: Motorola 16-bit at bytes 0-1; scaling * 0.1 - 100 = °C
            // CH assignment: CH1=FL, CH5=FR, CH9=RL, CH13=RR (NOT SURE ABOUT THIS)
            else if (msg.id == 0x4c4) {
                uint16_t raw = ((uint16_t)msg.data[0] << 8) | msg.data[1];
                vsm_state.izze_brake_fl = raw * 0.1f - 100.0f;
            } else if (msg.id == 0x4c5) {
                uint16_t raw = ((uint16_t)msg.data[0] << 8) | msg.data[1];
                vsm_state.izze_brake_fr = raw * 0.1f - 100.0f;
            } else if (msg.id == 0x4c6) {
                uint16_t raw = ((uint16_t)msg.data[0] << 8) | msg.data[1];
                vsm_state.izze_brake_rl = raw * 0.1f - 100.0f;
            } else if (msg.id == 0x4c7) {
                uint16_t raw = ((uint16_t)msg.data[0] << 8) | msg.data[1];
                vsm_state.izze_brake_rr = raw * 0.1f - 100.0f;
            }
            break;
        }
        default:
            break;
        }
    } // while (can.read(msg))
}

/**
 * @brief Sends the Drive, Traction, and Regen Modes CAN message
 *
 */
void sendCANMessage() {
    // Manual Bit packing
    uint8_t steering_data[] = {
        static_cast<uint8_t>(
            (driveState) |         // bits 0-1
            (tractionState << 2) | // bits 2-3
            (regenState << 4)      // bits 4-5
        )
    };

    CANMessage steering_msg{0x1b0, steering_data, 1};
    //printf("sending CAN message");
    can.write(steering_msg);
}

/**
 *  Called once every second
 */
void readFPS(){
    curFrame = screen.readFrames();
    fps = curFrame-lastFrame;
    lastFrame = curFrame;
    printf("fps: %d", fps);
}
