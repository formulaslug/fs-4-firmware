#include "BMS.h"

#include <algorithm>
#include <cmath>

BMS::BMS(CAN& CAN_POWERTRAIN, bool charging)
    : ltcBusInterface(&spiInterface), CAN_POWERTRAIN(CAN_POWERTRAIN) {

    chips.reserve(NUM_BATTERY_MODULES);
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        chips.emplace_back(ltcBusInterface, i);
    }
    // there should also be the startup checks for the ADCS on all the LTC6810s here. not a priority
    // but nice to havce

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) { // initlaize the tmp1075 handlers
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            tempSensors[i][j] =
                LTC6810::TMP1075_Handle_t{static_cast<uint8_t>(TMP1075_ADDRESSES[j]), 0x00};
        }
    }

    currentState = charging ? CHARGING : ACTIVE;
    Timer ltcTimeoutTimer;

    nBMS_Fault_3V3 = 1;
    packCurrent = 0;

}


void BMS::readCellVoltages() {
    // If PollTimeout is the only thing we've received for 100ms, throw a BMS fault.
    if (ltcTimeoutTimer.elapsed_time() >= 100ms) {
        currentState = FAULT;
        printf("ltcTimeoutTimer ran out! >100ms without successful reading! (only PollTimeouts)\n");
        return;
    }

    bool voltsConverted = true;
    LTC681xParallelBus::LTC681xBusStatus stat = ltcBusInterface.WakeupBus();
    LTC681xParallelBus::BusCommand command = LTC681xParallelBus::BuildBroadcastBusCommand(
        StartCellVoltageADC(AdcMode::k7k, false, CellSelection::kAll)
    );
    stat = ltcBusInterface.SendCommand(command);

    // TODO: Why does this seem to always fail with PollTimeout?
    // ThisThread::sleep_for(10ms);
    ThisThread::sleep_for(4ms);
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        command = LTC681xParallelBus::BuildAddressedBusCommand(PollADCStatus(), i);
        stat = ltcBusInterface.PollAdcCompletion(command, 1);

        if (stat == LTC681xBus::LTC681xBusStatus::PollTimeout) {
            printf("ADC poll timeout, on Module %d\n", i);
            //voltsConverted = false;
            ltcTimeoutTimer.start();
            // The rules require that we need to ensure we are getting data and that all sensors are
            // working correctly. If we cannot get an adc conversion in 100ms this will thow a
            // fault. That time period is a little arbitrary and probably should be adjusted
        }
    }

    ThisThread::sleep_for(3ms);

    if (voltsConverted) {
        ltcTimeoutTimer.stop();
        ltcTimeoutTimer.reset();
        // reset timer after successful adc conversions...
        for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
            uint8_t voltageReading[12] = {0};
            ltcBusInterface.WakeupBus();
            command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupA(), i);
            stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageReading);

            command = LTC681xParallelBus::BuildAddressedBusCommand(ReadCellVoltageGroupB(), i);
            stat = ltcBusInterface.SendReadCommand(command, (uint8_t*)voltageReading + 6);

            uint16_t* castVoltages = (uint16_t*)voltageReading;
            for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
                printf("%d\n", castVoltages[j]);
                voltages[i][j] = castVoltages[j];
            }
            // printf("\n");

            // 6 bytes per cell group reading (2 bytes per cell) ... transmitted in little endian
            // casted so that its easier to read
        }

        minCelVoltage = voltages[0][0];
        maxCellVoltage = voltages[0][0];
        packVoltageMv = 0;

        for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
            for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
                packVoltageMv += voltages[i][j];

                if (voltages[i][j] < minCelVoltage) {
                    minCelVoltage = voltages[i][j];
                }
                if (voltages[i][j] > maxCellVoltage) {
                    maxCellVoltage = voltages[i][j];
                }
            }
        }
    }
}

void BMS::readTemps() {
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            temps[i][j] = chips[i].readTemperatureTMP1075(&tempSensors[i][j]);
            printf("temperature ... tems %f\n", temps[i][j]);
        }
    }

    int8_t maxTemp = 0;

    // Find the hottest cell across all modules and sensors
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            if (temps[i][j] > maxTemp) {
                maxTemp = temps[i][j];
            }
        }
    }
    maxCellTemp = maxTemp;
}

// Turns on balancing for chips. Balancing should be done while charging and
// cells are most of the way charged, or when active and cells are most of the
// way charged. Our balancing threshold is at 85% of maximum cell voltage
void BMS::turnOnBalancing() {
    if (currentState != FAULT) {
            for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
                uint8_t dischargeValue = 0x00;
                LTC6810::Configuration& config = chips[i].getConfig();
                // uint16_t moduleVolts[NUM_VOLTAGES_PER_MODULE] = voltages[i];
                uint16_t minModuleVolt = voltages[i][0];
                uint16_t maxModuleVolt = voltages[i][0];
                for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
                    // if(voltages[i][j] < minModuleVolt){
                    //     minModuleVolt = voltages[i][j];
                    // }
                    if (voltages[i][j] > maxModuleVolt) {
                        maxModuleVolt = voltages[i][j];
                    }
                    if ((maxModuleVolt - minCelVoltage) >= DIFFERENCE_THRESHOLD) {
                        dischargeValue |= (0x1 << j); // we balance based on the whole battery
                        j++; // this ensures that a BMS will not balance adjacent cells
                    }
                    // Logic is find lowest voltage cell - go through each module and balance that
                    // module based on that cell reading so the whole battery is balanced
                }
                config.dischargeState.value = dischargeValue;
                chips[i].updateConfig();
            }
        balancing = 1;
    }
}

void BMS::turnOffBalancing() {
    ltcBusInterface.WakeupBus();
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        LTC6810::Configuration& config = chips[i].getConfig();
        config.dischargeState = {.value = 0};
        chips[i].updateConfig();
    }
    // printf("Cell balancing deactivated....\n");
    balancing = 0;
}

void BMS::readPackCurrent() {
    // HASS 300-S current sensor constants
    // (https://www.lem.com/sites/default/files/products_datasheets/hass-50_600-s-v22.pdf)

    // Nominal primary current (A)
    constexpr float HASS300_IPN = 300.0f;
    constexpr float HASS300_SENSITIVITY = 0.625f; // V/Ipn
    // "Ref" pin of the InAmps; i.e. output voltage at 0 current (V)
    constexpr float HASS300_INSTR_AMP_VREF = 0.2024f; // 0.174f;
    // Gain on InAmp output
    constexpr float HASS300_INSTR_AMP_GAIN = 1.796f; // 61.86 // 62.18

    float voutPos = V_Out_Positive.read() * 3.3f; // idk about this one
    float voutNeg = V_Out_Negative.read() * 3.3f;
    // HASS 300-S: I = (Vout - Vref) * IPN / 0.625

    float packCurrentAmpsOutput = (voutPos - HASS300_INSTR_AMP_VREF)
                                  / HASS300_SENSITIVITY
                                  * HASS300_IPN
                                  / HASS300_INSTR_AMP_GAIN;
    float packCurrentAmpsInput = (voutNeg - HASS300_INSTR_AMP_VREF)
                                 / HASS300_SENSITIVITY
                                 * HASS300_IPN
                                 / HASS300_INSTR_AMP_GAIN;

    // At very low current values the negative value jumps up crazy so we should
    // return a value of the total current based on the voltage difference
    if (voutPos >= voutNeg) {
        packCurrent = packCurrentAmpsOutput;
    } else {
        packCurrent = -1 * packCurrentAmpsInput;
    }

    // Keeping this here for debug purposes
    // printf(
    //     "Current sense Vout Positive: %.3f V  =>  Pack current (out of battery): %.2f \n",
    //     voutPos,
    //     packCurrentAmpsOutput
    // );
    // printf(
    //     "Current sense vout Negative: %.3f V => pack current (into battery) %.2f\n",
    //     voutNeg,
    //     packCurrentAmpsInput
    // );
}

void BMS::checkForFaults() {
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        // Cell Voltage Faults
        for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
            uint16_t voltVal = voltages[i][j];
            if (voltVal >= MAX_CELL_VOLTAGE || voltVal <= MIN_CELL_VOLTAGE) {
                currentState = FAULT;
                if (faultLoc == NONE) {
                    faultLoc = VOLTAGE;
                } else {
                    faultLoc = BOTH;
                }
                nBMS_Fault_3V3 = 0;
                faultModIndex = i;
                faultSenseIndex = j;
                if (voltVal >= MAX_CELL_VOLTAGE) {
                    cellTooHigh = 1;
                } else {
                    cellTooLow = 1;
                }
            }
        }

        // Cell Temp Faults
        const int8_t max_temp = currentState == CHARGING ? MAX_CELL_TEMP_CHARGING : MAX_CELL_TEMP;
        const int8_t min_temp = currentState == CHARGING ? MIN_CELL_TEMP_CHARGING : MIN_CELL_TEMP;

        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            int8_t tempReading = temps[i][j];
            if (currentState == CHARGING) {
                if (tempReading >= max_temp || tempReading <= min_temp) {
                    currentState = FAULT;
                    // this is for telemetry purposes
                    if (faultLoc == NONE) {
                        faultLoc = TEMPS;
                    } else {
                        faultLoc = BOTH;
                    }
                    nBMS_Fault_3V3 = 0;
                    faultModIndex = i;
                    faultSenseIndex = j;
                    if (tempReading >= max_temp) {
                        cellTooLow = 1;
                    } else {
                        cellTooHigh = 1;
                    }
                }
            }
        }
    }

    // Watch pack current. TODO: need to add negative here as well
    if (std::fabs(packCurrent) > MAX_PACK_CURRENT_AMPS) {
        // currentState = FAULT;
        // nBMS_Fault_3V3 = 0;
        printf(
            "ERROR: Overcurrent detected: %.2f A (not throwing actual BMS fault)\n", packCurrent
        );
    }

    // Check bms fault input, only for CAN logging (should really not be owned by BMS)
    imdFaultStat = !nIMD_Fault_3V3.read();
}

// i am undecided wether or not to put the bms into a fault state here - might add a state that
// doesnt turn on any bms indicator lights but essentially stops the bms functions like in a fault
// state not totally sure whats required of the bms in this case making a best guess
/*
    only one we care about is the final shutdown circuit reading - we only care about it for the
   purposes of precharing again after a shutdown

    the other shutdown circuit inputs should be reported to can 4-4-26


*/

void BMS::controller() {

    if (currentState != FAULT) {




        // readTemps();
        // readCellVoltages();

        // const bool can_balance = maxCellVoltage >= BALANCING_THRESHOLD || currentState == CHARGING;
        // if (can_balance) {
        //     turnOnBalancing();
        //     ThisThread::sleep_for(100ms);
        //     turnOffBalancing();
        // }

        // readCellVoltages();
        readTemps();
        printf("\n\n\n\n\n");

        // for (int i = 0; i < NUM_BATTERY_MODULES; i++) {
        //     for (int j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
        //         printf("Voltage: Module %d, Cell %d: %d mv\n", i, j, voltages[i][j]);
        //     }
        // }
        // // for (int i = 0; i < NUM_BATTERY_MODULES; i++) {
        // //     for (int j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
        // //         printf("Temp: Module %d, Sensor %d: %f degC\n", i, j, temps[i][j]);
        // //     }
        // // }

        for (int i = 0; i < NUM_BATTERY_MODULES; i++) {
            auto& config = chips[i].getConfig();
            config.gpio1 = LTC6810::GPIOOutputState::kLow;
            chips[i].updateConfig();
        }

        // checkForFaults();
        // if (currentState == FAULT) return;

        // readPackCurrent();
        // printf("Pack current: %f A\n", packCurrent);

    } else {
        printf("BMS: FAULT STATE\n");
        turnOffBalancing();
        nBMS_Fault_3V3 = 0;
        // need to turn on indicator lights as well .....
        TS_READY = 0;
    }
}
