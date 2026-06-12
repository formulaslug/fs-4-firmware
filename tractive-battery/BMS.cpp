#include "BMS.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstdint>

BMS::BMS(CAN& CAN_POWERTRAIN, bool charging, Mutex& mainMutex)
    : ltcBusInterface(&spiInterface), CAN_POWERTRAIN(CAN_POWERTRAIN), TelemetryLock(mainMutex) {

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

    // get initial values
    turnOffBalancing();
    readCellVoltages();
    readTemps();
    checkForFaults();
}

void BMS::readCellVoltages() {
    // If PollTimeout is the only thing we've received for 100ms, throw a BMS fault.
    if (ltcTimeoutTimer.elapsed_time() >= 500ms) {
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
            voltsConverted = false;
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
                // printf("%d\n", castVoltages[j]);
                voltages[i][j] = castVoltages[j] / 10;
            }
            // printf("\n");

            // 6 bytes per cell group reading (2 bytes per cell) ... transmitted in little endian
            // casted so that its easier to read
        }

        uint16_t temp_minCelVoltage = 65535;
        uint16_t temp_maxCellVoltage = 0;
        uint32_t temp_packVoltageMv = 0;

        for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
            for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
                temp_packVoltageMv += voltages[i][j];

                if (voltages[i][j] < temp_minCelVoltage) {
                    temp_minCelVoltage = voltages[i][j];
                }
                if (voltages[i][j] > temp_maxCellVoltage) {
                    temp_maxCellVoltage = voltages[i][j];
                }
            }
        }

        minCellVoltage = temp_minCelVoltage;
        maxCellVoltage = temp_maxCellVoltage;
        packVoltageMv = temp_packVoltageMv;
    }
}

void BMS::readTemps() {

    for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
        uint8_t commData[6] = {0};

        uint8_t temp_sense_address = tempSensors[0][j].i2c_address;
        uint8_t temp_sense_reg = tempSensors[0][j].temp_reg;

        // STEP 1: Write register pointer (tell sensor which register to read)
        // I2C Sequence: START -> [ADDR+W] -> ACK -> [REG] -> STOP

        // BYTE 0: Address with Write bit (ADDR << 1|0)
        commData[0] = (0x6 << 4) | ((((temp_sense_address << 1) | 0x00) >> 4) & 0x0F);
        commData[1] = ((((temp_sense_address << 1) | 0x00) & 0x0F) << 4) | 0x0;

        // BYTE 1: Register address (0x00 for temperature register)
        // FCOM 0X9 (Master NACK + STOP)
        commData[2] = (0x1 << 4) | (((temp_sense_reg) >> 4) & 0x0F);
        commData[3] = (((temp_sense_reg) & 0x0F) << 4) | 0x9;

        // BYTE 2: Dummy (not used)
        commData[4] = (0x7 << 4) | ((0x00 >> 4) & 0x0F);
        commData[5] = ((0x00 & 0x0F) << 4) | 0x0;

        ltcBusInterface.WakeupBus();
        LTC681xParallelBus::BusCommand wrCmd =
            LTC681xParallelBus::BuildBroadcastBusCommand(WriteCommGroup());
        ltcBusInterface.SendDataCommand(wrCmd, commData);

        ThisThread::sleep_for(3ms);

        // The STCOMM command is to be followed by 24 clock
        // cycles for each byte of data to be transmitted to the slave
        // device while holding CSB low. For example, to transmit 3
        // bytes of data to the slave, send STCOMM command and
        // its PEC followed by 72 clock cycles. Pull CSB high at the
        // end of the 72 clock cycles of STCOMM command.
        ltcBusInterface.WakeupBus();
        LTC681xParallelBus::BusCommand stCmd =
            LTC681xParallelBus::BuildBroadcastBusCommand(StartComm());
        static uint8_t buf[6] = {};
        ltcBusInterface.SendDataCommand(stCmd, buf);

        ThisThread::sleep_for(3ms);

        // STEP 2: Read 2 bytes from temperature register
        // I2C Sequence: START -> [ADDR+R] -> ACK -> [READ MSB] -> ACK -> [READ LSB] -> NACK+STOP
        // BYTE 0: Address with Read bit (ADDR << 1 |1)
        commData[0] = (0x6 << 4) | ((((temp_sense_address << 1) | 0x01) >> 4) & 0x0F);
        commData[1] = ((((temp_sense_address << 1) | 0x01) & 0x0F) << 4) | 0x0;

        // BYTE 1: Read MSB (master sends 0xFF as dummy, master ACKs)
        commData[2] = (0x0 << 4) | ((0xFF >> 4) & 0x0F);
        commData[3] = ((0XFF & 0x0F) << 4) | 0x0;

        // BYTE 2: Read LSB (master sends 0xFF as dummy, master NACKs and STOP)
        commData[4] = (0x0 << 4) | ((0xFF >> 4) & 0x0F);
        commData[5] = ((0XFF & 0x0F) << 4) | 0x9;

        ltcBusInterface.WakeupBus();
        ltcBusInterface.SendDataCommand(wrCmd, commData);
        ltcBusInterface.SendDataCommand(stCmd, buf);

        ThisThread::sleep_for(3ms);

        for (int i = 0; i < NUM_BATTERY_MODULES; i++) {
            uint8_t rxData[8] = {0};

            ltcBusInterface.WakeupBus();
            auto rdCmd = LTC681xBus::BuildAddressedBusCommand(ReadCommGroup(), i);
            ltcBusInterface.SendReadCommand(rdCmd, rxData);

            // STEP 3: Extract Temperature Data from COMM register
            // MSB is in D1 (rxData[2] upper nibble, rxData[3] lower nibble)

            uint8_t tempMSB = ((rxData[2] & 0x0F) << 4) | ((rxData[3] >> 4) & 0x0F);

            // LSB is in comm register 3 4 msbs
            uint8_t tempLSB = (rxData[3] & 0xF0) >> 4;

            // Combine MSB and LSB into 16-bit value

            int16_t rawTemp = (tempMSB << 4) | (tempLSB & 0x0f);

            // STEP 4: Convert to Celsius
            // TMP1075-specific conversion:
            // 12-bit temperature value stored in bits 15-4
            // Each LSB = 0.0625°C
            temps[i][j] = ((float)rawTemp) * 0.0625f;
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

// Turns on balancing for all chips. Balancing should be done while charging and
// cells or when active and cells are most of the way charged. Our balancing
// threshold is at 85% of maximum cell voltage. Should only be called when there
// is no fault.
//
// From the LTC6810 datasheet:
// "The LTC6810 does not allow adjacent discharge switches to be asserted, so
// the WRFG command will not be executed if adjacent DCC bits in the CONFIG
// register are asserted. The current path shown at the right of Figure 40b
// shows that if adjacent discharges switches were permitted to be on, discharge
// current would flow through the series combination of cells instead of the
// individual cells."
void BMS::turnOnBalancing() {
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        LTC6810::Configuration& config = chips[i].getConfig();

        uint8_t dischargeValue = 0x00;
        uint8_t cellVisited = 0x00;

        for (uint8_t k = 0; k < NUM_VOLTAGES_PER_MODULE; k++) {
            int8_t maxVoltageIndex = -1; // -1 is not found yet
            uint16_t maxVolt = 0;

            for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
                if (cellVisited & (1 << j)) {
                    continue;
                }

                if (voltages[i][j] > maxVolt) {
                    maxVoltageIndex = j;
                    maxVolt = voltages[i][j];
                }
            }

            if (maxVoltageIndex == -1) {
                break;
            }

            if (voltages[i][maxVoltageIndex] < BALANCING_THRESHOLD) {
                break;
            }

            cellVisited |= (1 << maxVoltageIndex);

            bool neighborIsDischarging = false;

            if (maxVoltageIndex > 0) {
                if (dischargeValue & (1 << (maxVoltageIndex - 1))) {
                    neighborIsDischarging = true;
                }
            }
            if (maxVoltageIndex < (NUM_VOLTAGES_PER_MODULE - 1)) {
                if (dischargeValue & (1 << (maxVoltageIndex + 1))) {
                    neighborIsDischarging = true;
                }
            }

            if (!neighborIsDischarging
                && voltages[i][maxVoltageIndex] - minCellVoltage >= DIFFERENCE_THRESHOLD)
            {
                dischargeValue |= (1 << maxVoltageIndex);
            }
        }

        config.dischargeState.value = dischargeValue;
        // printf("%d: %s\n", i, std::bitset<8>(dischargeValue).to_string().c_str());
        ltcBusInterface.WakeupBus();
        chips[i].updateConfig();
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
}

void BMS::readPackCurrent() {
    // HASS 300-S current sensor constants
    // (https://www.lem.com/sites/default/files/products_datasheets/hass-50_600-s-v22.pdf)

    // Nominal primary current (A)
    constexpr float HASS300_IPN = 300.0f;
    constexpr float HASS300_SENSITIVITY = 0.625f; // V/Ipn

    // Todo: tune these more
    // "Ref" pin of the InAmps; i.e. output voltage at 0 current (V)
    constexpr float HASS300_INSTR_AMP_VREF = 0.2024f; // 0.174f;
    // "Gain" pin of the InAmps.
    constexpr float HASS300_INSTR_AMP_GAIN = 1.69f; // 1.796f;

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
    bool faultPresent = false;
    uint8_t modFaultIndex = 0;
    uint8_t componentFaultIndex = 0;
    for (uint8_t i = 0; i < NUM_BATTERY_MODULES; i++) {
        // Cell Voltage Faults
        for (uint8_t j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
            uint16_t voltage = voltages[i][j];
            if (voltage >= MAX_CELL_VOLTAGE_MV || voltage <= MIN_CELL_VOLTAGE_MV) {
                // printf("voltage fault: (%d, %d): %d\n", i, j, voltage);
                modFaultIndex = i;
                componentFaultIndex = j;
                faultPresent = true;
                if (voltage >= MAX_CELL_VOLTAGE_MV) {
                    cellVoltageTooHigh = 1;
                } else {
                    cellVoltageTooLow = 1;
                }
            }
        }

        // Cell Temp Faults
        const int8_t MAX_TEMP = currentState == CHARGING ? MAX_CELL_TEMP_CHARGING : MAX_CELL_TEMP;
        const int8_t MIN_TEMP = currentState == CHARGING ? MIN_CELL_TEMP_CHARGING : MIN_CELL_TEMP;

        for (uint8_t j = 0; j < NUM_TEMP_SENSORS_PER_MODULE; j++) {
            int8_t tempReading = temps[i][j];
            if (tempReading >= MAX_TEMP) {
                // throwFault(i, j);
                // printf("overtemp: (%d, %d): %d\n", i, j, tempReading);
                modFaultIndex = i;
                componentFaultIndex = j;
                faultPresent = true;
                if (currentState == CHARGING) {
                    packTempTooHighCrg = 1;
                } else {
                    packTempTooHigh = 1;
                }
            } else if (tempReading <= MIN_TEMP) {
                // throwFault(i, j);
                // printf("undertemp: (%d, %d): %d\n", i, j, tempReading);
                modFaultIndex = i;
                componentFaultIndex = j;
                faultPresent = true;
                packTempTooLow = 1;
            }
        }
    }

    if (faultPresent) {
        faultCounter++;
    } else {
        faultCounter = 0;
    }

    if (faultCounter >= FAULT_LIMIT) {
        throwFault(modFaultIndex, componentFaultIndex);
    }
}

// Change BMS State to FAULT, set nBMS_Fault pin, set fault sensor indexes
void BMS::throwFault(int moduleIndex, int sensorIndex) {
    currentState = FAULT;
    nBMS_Fault_3V3 = 0;
    faultModuleIndex = moduleIndex;
    faultSensorIndex = sensorIndex;
}

// Main controller loop (to be called periodically at high frequency)
void BMS::controller() {
    turnOffBalancing();
    readCellVoltages();
    for (int i = 0; i < NUM_BATTERY_MODULES; i++) {
        for (int j = 0; j < NUM_VOLTAGES_PER_MODULE; j++) {
            printf("c[%d][%d]: %f mv  ", i, j, temps[i][j]);
        }
        printf("\n");
    }
    const bool can_balance = maxCellVoltage >= BALANCING_THRESHOLD || currentState == CHARGING;
    if (currentState != FAULT && can_balance) {
        turnOnBalancing();
        balancing = true;
    } else {
        balancing = false;
    }
    printf("pack: %d mv\n", packVoltageMv);
    readTemps();
    checkForFaults();
    readPackCurrent();

    if (currentState == FAULT) {
        printf("BMS: FAULT STATE\n");
        turnOffBalancing();
        nBMS_Fault_3V3 = 0;
    }
}
