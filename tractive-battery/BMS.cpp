#include "BMS.h"
#include <algorithm>
#include <cmath>

// ideally I would want thse in BMS.h for now its fine tho.
//  static constexpr float ADC_REF_VOLTAGE = 3.3f;
//  //this will probably be adjusted and tuned as testing happens
//  static constexpr float CURRENT_SENSOR_VOLTS_PER_AMP = 0.0037f; // first-pass estimate
//  static constexpr size_t CURRENT_SENSOR_CALIBRATION_SAMPLES = 500;
//  static constexpr float MAX_PACK_CURRENT_AMPS = 1000.0f; // adjust later

BMS::BMS()
    : ltcBusInterface(&spiInterface) // not a huge fan of this but i think its required
{
    currentState = ACTIVE; // assume everything is okay at startup
    Timer ltcTimeoutTimer; // timer for ltc6810 timeout
    CANMessage msg;        // can message object
    nBMS_Fault_3V3 = 1;    // assume no fault at startup
    nPrechargeControl = 1; // no precharge during startup

    currentSensorOffsetVolts = 0.0f;
    packCurrentAmps = 0.0f;
    currentSensorCalibrated = false;

    if (Charge_State_Filtered.read()) {
        currentState = CHARGING;
    } else {
        currentState = ACTIVE;
    }
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

void BMS::chargingActions() {
    // current work in progress - needs to detect soc over can......
    // I have several questions about this ... do we fault based on this data what exactly do we do
    // with it - might reference fs3 code from fs3 adapted to fs4
    uint8_t canData = 0;
    CAN_POWERTRAIN.read(msg);
    uint32_t id = msg.id;
    unsigned char* data = msg.data;

    if (!(currentState == FAULT)) {
        switch (id) {
        case 0x682: // temperature message from MC
            canData = (data[2] | (data[3] << 8));
            break;
        default:
            break;
        }
    } else {
        switch (id) {
        case 0x190: // charge status from charger, 180 + node ID (10)
            canData = (data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)) / 100;
        default:
            break;
        }
    }
}

void BMS::turnOffCellBalancing() {
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        LTC6810::Configuration& config = chips[i].getConfig();
        config.dischargeState = {.value = 0};
        chips[i].updateConfig();
    }
    printf("Cell balancing deactivated....\n");
}

void BMS::readCellVoltages() {
    // char msg3[] = "Reading Cell Voltages\n";
    // VCP_UART.write(msg3, sizeof(msg3));
    printf("Reading cell voltages...\n");
    printf("elapsed time ... \n");
    printf("%f\n", ltcTimeoutTimer.elapsed_time());
    // ltcBusInterface.WakeupBus();
    if (ltcTimeoutTimer.elapsed_time() >= 100ms) {
        currentState = FAULT;
        return;
    }

    bool voltsConverted = true;
    LTC681xParallelBus::LTC681xBusStatus stat = ltcBusInterface.WakeupBus();
    LTC681xParallelBus::BusCommand command = LTC681xParallelBus::BuildBroadcastBusCommand(
        StartCellVoltageADC(AdcMode::k7k, false, CellSelection::kAll)
    );
    stat = ltcBusInterface.SendCommand(command);

    ThisThread::sleep_for(3ms);

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        command = LTC681xParallelBus::BuildAddressedBusCommand(PollADCStatus(), i);
        stat = ltcBusInterface.PollAdcCompletion(command, 0);

        if (stat == LTC681xBus::LTC681xBusStatus::PollTimeout) {
            // printf("ADC poll timeout, on Bank %d\n", i);
            voltsConverted = false;
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

        printf("read voltages...\n");
        minVoltage = voltages[0][0];
        maxVoltage = voltages[0][0];

        for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
            for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
                if (voltages[i][j] < minVoltage) {
                    minVoltage = voltages[i][j];
                }
                if (voltages[i][j] > maxVoltage) {
                    maxVoltage = voltages[i][j];
                }
            }
        }
    }
}

void BMS::readTemps() {

    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            cellTemps[i][j] = chips[i].readTemperatureTMP1075(&sensors[i][j]);
        }
    }

    int8_t maxTemp = 0;

    // Find the hottest cell across all modules and sensors
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            if (cellTemps[i][j] > maxTemp) {
                maxTemp = cellTemps[i][j];
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
        if (maxVoltage >= BALANCING_THRESHOLD && minVoltage > MIN_CELL_VOLTAGE) {
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
                    if ((maxModuleVolt - minVoltage) >= DIFFERENCE_THRESHOLD) {
                        dischargeValue |= (0x1 << j); // we balance based on the whole battery
                    }
                    /*
                        logic is find lowest voltage cell - go through each module and balance that
                       module based on that cell reading so the whole battery is balanced

                    */
                }
                config.dischargeState.value = dischargeValue;
                chips[i].updateConfig();
            }
        }
    }
}

void BMS::readPackCurrent() {
    float vout = V_Out_Positive.read() * HASS300_ADC_REF;

    // HASS 300-S: I = (Vout - Vref) * IPN / 0.625
    packCurrentAmps = (vout - HASS300_VREF) / HASS300_SENSITIVITY;

    printf("Current sense Vout: %.3f V  =>  Pack current: %.2f A\n", vout, packCurrentAmps);
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
                nBMS_Fault_3V3 = 0;
            }
        }

        // cell temp based faults...
        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            int8_t tempReading = cellTemps[i][j];
            if (currentState == CHARGING) {
                if (tempReading >= CHARGING_CELL_MAX) {
                    currentState = FAULT;
                    nBMS_Fault_3V3 = 0;
                } else if (tempReading <= CHARGING_CELL_MIN) {
                    currentState = FAULT;
                    nBMS_Fault_3V3 = 0;
                }
            }
        }
    }

    // tray temp sensor checks
    //  for testing purposes i am going to use the cell temperature limits here, will be update
    //  later
    // telemetry stuff needs to be added but the logic is there.

    for (uint8_t i = 0; i < NUM_TRAY_TEMP_SENSORS;
         i++) { // keeping this here for now but its mainily a telemetrey
        uint8_t trayTemp = trayTemps[i];
        if (trayTemp >= CELL_MAX || trayTemp <= CELL_MIN) {
            // should not cause a fault just for data 4-4-26 - keeping it here for mow
            //  currentState = FAULT;
            //  nBMS_Fault_3V3 = 0;
        }
    }

    // to watch pack current
    if (std::fabs(packCurrentAmps) > MAX_PACK_CURRENT_AMPS) {
        currentState = FAULT;
        nBMS_Fault_3V3 = 0;
        printf("FAULT: overcurrent detected: %.2f A\n", packCurrentAmps);
    }

    // to check IMDStatus.....
    //  faults on high reading i think - double check
    //  i believe this opens the shutdown cirtui

    if (IMD_Fault_3V3.read() == 0) {
        printf("IMD FAULT READ ... \n");
        // this will open the shutdown circuit on its own so i think its mostly a telemetry thing
    }
}

// i am undecided wether or not to put the bms into a fault state here - might add a state that
// doesnt turn on any bms indicator lights but essentially stops the bms functions like in a fault
// state not totally sure whats required of the bms in this case making a best guess
/*
    only one we care about is the final shutdown circuit reading - we only care about it for the
   purposes of precharing again after a shutdown

    the other shutdown circuit inputs should be reported to can 4-4-26


*/

// void BMS::checkShutdownCircuit(){
//     if(Shutdown_In_3V3_Filtered.read()==0 || Shutdown_Out_3V3_Filtered.read()==0){
//         // shutdown circuit open before the bms
//         turnOffCellBalancing();
//         //precharge should also be turned off but thats in a different task
//         if(Shutdown_Out_3V3_Filtered.read()==0){
//             //shutdown circuit opened after the bms
//             turnOffCellBalancing();
//         }
//         if(Shutdown_In_3V3_Filtered.read()==0){
//             turnOffCellBalancing();
//         }
//     }
// }

void BMS::controlFans() {
    int8_t maxTemp = maxCellTemp;

    if (currentState != PRECHARGING) {
        // Linear scaling: 20% at ~20°C, 100% at ~50°C
        // Formula: (2.6667 * temp) - 33.3333, clamped to [20, 100]
        int raw_percent = (int)((2.6667f * maxTemp) - 33.3333f);
        uint8_t fan_percent = (uint8_t)std::clamp(raw_percent, 20, 100);
        Fan_PWM.write(fan_percent / 100.0f); // PWM expects 0.0 - 1.0
    } else {
        // Keep fans off until precharge is complete
        Fan_PWM.write(0.0f);
    }
}

void BMS::controller() {

    if (currentState != FAULT) {

        chargingActions();
        printf("charging actions completed okay...\n");
        turnOffCellBalancing();
        printf("turn off cell balancing completed okay...\n");
        ThisThread::sleep_for(3ms);
        readCellVoltages();
        printf("cellvoltages read okay\n");
        // readTemps();
        printf("read temps went okay....\n");
        checkForFaults();
        printf("checked for faults\n"); //
        controlFans();
        printf("fan pwm set ...\n");
        decideBalancing();
        printf("battery balancing set....");
        // d266270a (testing updates)
        // checkShutdownCircuit(;
    } else {
        printf("WE ARE IN FAULT");
        turnOffCellBalancing();
        nBMS_Fault_3V3 = 0;
        // need to turn on indicator lights as well .....
        //  precharge relay should be open in this case
        // fans turn off on fault
        Fan_PWM.write(0.0f);
    }
}
