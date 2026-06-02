#include "BMS.h"

#include <algorithm>
#include <cmath>

// ideally I would want thse in BMS.h for now its fine tho.
//  static constexpr float ADC_REF_VOLTAGE = 3.3f;
//  //this will probably be adjusted and tuned as testing happens
//  static constexpr float CURRENT_SENSOR_VOLTS_PER_AMP = 0.0037f; // first-pass estimate
//  static constexpr size_t CURRENT_SENSOR_CALIBRATION_SAMPLES = 500;
//  static constexpr float MAX_PACK_CURRENT_AMPS = 1000.0f; // adjust later

BMS::BMS(CAN& CAN_POWERTRAIN, bool charging)
    : ltcBusInterface(&spiInterface), CAN_POWERTRAIN(CAN_POWERTRAIN) {

    chips.reserve(NUM_BATTERY_MODULES);
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        chips.emplace_back(ltcBusInterface, i);
    }
    // CAN_POWERTRAIN = CAN_POWERTRAIN;

    // there should also be the startup checks for the ADCS on all the LTC6810s here. not a priority
    // but nice to havce

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) { // initlaize the tmp1075 handlers
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            tempSensors[i][j] =
                LTC6810::TMP1075_Handle_t{static_cast<uint8_t>(TMP1075_ADDRESSES[j]), 0x00};
        }
    }

    currentState = ACTIVE; // assume everything is okay at startup
    Timer ltcTimeoutTimer; // timer for ltc6810 timeout
    nBMS_Fault_3V3 = 1;    // assume no fault at startup

    packCurrentAmpsOutput = 0.0f;
    packCurrentAmpsInput = 0.0f;

    // intialize data - assume everything is good at startup

    // Data = {false, false, false, true, true, true, false, false, false, false, false, false,
    // false, false, 0,0,0,0,0};
}

/*
    move structs and telemetry data to seperate can class - call can class every so often using data
   from BMS instance move BMSfaultDetection functions to BMS class so we dont have to keep passing a
   reference (in the event queue we would make controller a public function and call that ) make
   changes to precharge ... soon - note ideally precharge runs once per startup we do NOT close the
   precharge relay while the car is running (add real tmp1075 addresses)




    note for precharge - precharge should be run once during startup and once after each time the
   shutdown circuit is open

    - implementing current sensor is underway make sure changes are announced beforehand
*/

void BMS::turnOffCellBalancing() {
    ltcBusInterface.WakeupBus();
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        LTC6810::Configuration& config = chips[i].getConfig();
        config.dischargeState = {.value = 0};
        chips[i].updateConfig();
    }
    printf("Cell balancing deactivated....\n");
    balancing = 0;
}

void BMS::readCellVoltages() {
    // ltcBusInterface.WakeupBus();
    if (ltcTimeoutTimer.elapsed_time() >= 100ms) {
        // currentState = FAULT;
        // return;
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
            // printf("ADC poll timeout, on Bank %d\n", i);
            // voltsConverted = false;
            ltcTimeoutTimer.start();
            printf("poll timeout occured...\n");
            // the rules require that we need to ensure we are getting data and that all sensors are
            // working correctly, if we cannot get an adc conversion in 100ms this will thow a fault
            // that time period is a little arbitrary and probably should be adjusted
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
                // volt = castVoltages[j];
                printf("%d\n", castVoltages[j]); // printing the cell voltages for testing purposes

                voltages[i][j] = castVoltages[j];
            }
            printf("\n");
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

// balancing should be done while charging and most cells are most of the way charged, or when idle
// and most cells are mostly charged our balancing threshold is at 85% of maximum charges
void BMS::decideBalancing() {
    // turns on balancing for chips
    printf("deciding balancing......");
    if (currentState != FAULT) {
        if (maxCellVoltage >= BALANCING_THRESHOLD && minCelVoltage > MIN_CELL_VOLTAGE) {
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
                    }
                    // Logic is find lowest voltage cell - go through each module and balance that
                    // module based on that cell reading so the whole battery is balanced
                    // it wont let us do adjacent cells so .... may need to update ..
                }
                config.dischargeState.value = dischargeValue;
                chips[i].updateConfig();
            }
        }
        balancing = 1;
    }
}

void BMS::readPackCurrent() {
    // HASS 300-S current sensor constants (https://www.lem.com/sites/default/files/products_datasheets/hass-50_600-s-v22.pdf)

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
    packCurrentAmpsOutput = (voutPos - HASS300_INSTR_AMP_VREF) / HASS300_SENSITIVITY * HASS300_IPN / HASS300_INSTR_AMP_GAIN;
    packCurrentAmpsInput = (voutNeg - HASS300_INSTR_AMP_VREF) / HASS300_SENSITIVITY * HASS300_IPN / HASS300_INSTR_AMP_GAIN;
    // not sure about the above putting this in here...
    //  packCurrentAmpsOutput =

    printf(
        "Current sense Vout Positive: %.3f V  =>  Pack current (out of battery): %.2f \n",
        voutPos,
        packCurrentAmpsOutput
    );
    printf(
        "Current sense vout Negative: %.3f V => pack current (into battery) %.2f\n",
        voutNeg,
        packCurrentAmpsInput
    );
}

void BMS::checkForFaults() {

    // we will need to modify this for telemetry purposes ie which module is faulting which
    // parrallel group is faulting etccc
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        // cell voltage based faults...
        for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
            uint16_t voltVal = voltages[i][j];
            if (voltVal >= MAX_CELL_VOLTAGE || voltVal <= MIN_CELL_VOLTAGE) {
                currentState = FAULT;
                if(faultLoc == NONE){
                    faultLoc = VOLTAGE;
                }else{
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

        // cell temp based faults...
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            int8_t tempReading = temps[i][j];
            if (currentState == CHARGING) {
                if (tempReading >= CHARGING_CELL_MAX_TEMP || tempReading <= CHARGING_CELL_MIN_TEMP)
                {
                    currentState = FAULT;
                    //this is for telemetry purposes
                    if(faultLoc == NONE){
                        faultLoc = TEMPS;
                    }else{
                        faultLoc = BOTH;
                    }
                    nBMS_Fault_3V3 = 0;
                    faultModIndex = i;
                    faultSenseIndex = j;
                    if (tempReading >= CHARGING_CELL_MIN_TEMP) {
                        cellTooLow = 1;
                    } else {
                        cellTooHigh = 1;
                    }
                }
            }
        }
    }

    // to watch pack current // need to add negative here as well
    if (std::fabs(packCurrentAmpsOutput) > MAX_PACK_CURRENT_AMPS) {
        currentState = FAULT;
        nBMS_Fault_3V3 = 0;
        printf("FAULT: overcurrent detected: %.2f A\n", packCurrentAmpsOutput);
    }

    // to check IMDStatus.....
    //  faults on high reading i think - double check
    //  i believe this opens the shutdown cirtui

    // if (currentState == FAULT) {
    //     bmsFault = 1;
    // }
}

// i am undecided wether or not to put the bms into a fault state here - might add a state that
// doesnt turn on any bms indicator lights but essentially stops the bms functions like in a fault
// state not totally sure whats required of the bms in this case making a best guess
/*
    only one we care about is the final shutdown circuit reading - we only care about it for the
   purposes of precharing again after a shutdown

    the other shutdown circuit inputs should be reported to can 4-4-26


*/

// void BMS::telemetryPins() {
//     Data.shutdownIn = Shutdown_In_3V3_Filtered.read();
//     Data.shutdownOut = Shutdown_Out_3V3_Filtered.read();
//     Data.shutdownFinal = Shutdown_Final_3V3_Filtered.read();
//     Data.imdStatus = IMD_Fault_3V3.read();
// }



void BMS::controller() {

    if (currentState != FAULT) {
        glvVoltage = (GLV_Voltage.read() * 3.3 * 21.9 / 3.9 * 1000);
        imdFaultStat = IMD_Fault_3V3.read(); 
        // chargingActions();
        // printf("charging actions completed okay...\n");
        // turnOffCellBalancing();
        // printf("turn off cell balancing completed okay...\n");
        // ThisThread::sleep_for(3ms);
        readCellVoltages();
        // printf("cellvoltages read okay\n");
        // turnOffCellBalancing();

        readTemps();
        for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
            for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
                printf("temp reading: ... %f\n", temps[i][j]);
            }
        }

        readPackCurrent();
        printf("\n\n");
        // printf("read temps went okay....\n");
        // checkForFaults();
        // printf("checked for faults\n"); //
        // controlFans();
        // printf("fan pwm set ...\n");
        // decideBalancing();
        // printf("battery balancing set....");
        // telemetryPins();
        // readPackCurrent();

    } else {
        printf("WE ARE IN FAULT");
        turnOffCellBalancing();
        nBMS_Fault_3V3 = 0;
        // need to turn on indicator lights as well .....
        //  precharge relay should be open in this case
        // fans turn off on fault
        // Fan_PWM.write(0.0f);
    }
}
