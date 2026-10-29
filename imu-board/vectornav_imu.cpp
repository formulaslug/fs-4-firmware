#include "vectornav_imu.h"
#include "vectornav/Implementation/MeasurementDatatypes.hpp"
#include "vectornav/Interface/Registers.hpp"
#include <cstdio>

VectorNavIMU::VectorNavIMU(PinName tx, PinName rx, PinName restore)
    : tx(tx), rx(rx), restore(restore, 0) {

    // Fixed: originally 100Hz (IMU), 25Hz (INS) and 100Hz (attitude),
    // original the rate comments had the math wrong
    // Changed so accel/gyro run at the full 800Hz IMU rate and attitude/INS at the
    // VN-200's 400Hz max in one packet
    imu_reg.asyncMode.emplace();
    imu_reg.asyncMode->serial1 = false;
    imu_reg.asyncMode->serial2 = true;
    imu_reg.rateDivisor = 1; // 800Hz / 1 = 800Hz
    imu_reg.imu.accel = true;
    imu_reg.imu.angularRate = true;

    nav_reg.asyncMode.emplace();
    nav_reg.asyncMode->serial1 = false;
    nav_reg.asyncMode->serial2 = true;
    nav_reg.rateDivisor = 2; // 800Hz / 2 = 400Hz
    nav_reg.attitude.ypr = true;
    // nav_reg.attitude.yprU = true;
    nav_reg.ins.posLla = true;
    // nav_reg.ins.posU = true;
    nav_reg.ins.velBody = true;
    // nav_reg.ins.velU = true;
    nav_reg.gnss.gnss1Fix = true;
}

uint32_t VectorNavIMU::baud() const {
    auto baud = sensor.connectedBaudRate();
    return baud.has_value() ? static_cast<uint32_t>(*baud) : 0;
}

VN::Error VectorNavIMU::start() {
    configure_attempts++;
    last_configure = Kernel::Clock::now();

    if (!connected) {
        restore_factory_settings();
        VN::Error err = sensor.connect(tx, rx, DEFAULT_BAUD);
        if (err != VN::Error::None) record_error(err);
        return_if_vn_error(err);
        connected = true;
        link_state = VectornavLink::Configuring;
        printf("Connected to sensor!\n");

        // Fixed: the sensor varies 4-6s to restart after factory reset. Asking it
        // anything while resetting gets answered late, so wait for its factory-default ASCII output
        // instead
        auto give_up = Kernel::Clock::now() + 10s;
        while (!sensor.getNextMeasurement(false) && Kernel::Clock::now() < give_up) {
            ThisThread::sleep_for(10ms);
        }
        while (sensor.getNextMeasurement(false)) {
        }
        last_configure = Kernel::Clock::now();
    }

    VN::Error err = configure();
    configured = (err == VN::Error::None);
    if (configured) {
        last_packet = Kernel::Clock::now();
        link_state = VectornavLink::NoData;
    } else {
        record_error(err);
    }
    return err;
}

// Fixed: when only the STM32 reset or boots up, the sensor was still at whatever baud it was last
// switched to. The VN-200 restores its factory settings (115200 baud) if RESTORE is high while it
// starts up, so hold RESTORE high and reset the sensor. We don't know its baud yet, so send the
// reset at both.
void VectorNavIMU::restore_factory_settings() {
    restore = 1;
    {
        BufferedSerial port{tx, rx};
        for (BaudRates baud : {FAST_BAUD, DEFAULT_BAUD}) {
            port.set_baud(static_cast<int>(baud));
            port.write(
                "$VNRST*XX\r\n", 11
            ); //$VNRST is the reset command *XX forces IMU to skip checksum
            port.sync();
        }
    }
    ThisThread::sleep_for(2500ms);
    restore = 0;
    printf("Restored sensor factory settings\n");
}

// Fixed: sensor is at DEFAULT_BAUD after it powers up, but still at FAST_BAUD if only this
// board reset, so if it doesn't answer at our current baud, try the other one. You can test
// this capability on screen, just hit reset button on STM32 after its been running. The IMU
// is at higher baudrate (already been switched), but STM32 tries at 115200 (fails) and then
// switches automatically to 921600. Eventually switch to force reset using RESTORE Pin
VN::Error VectorNavIMU::find_sensor() {
    // Poll and print the model number using a read register command
    // Create an empty register object of the necessary type, where the data member will be populated when the sensor responds to our "read register" request
    VN::Registers::System::Model model_register;
    VN::Error err = sensor.readRegister(&model_register, true);
    if (err != VN::Error::None) {
        BaudRates other = (baud() == static_cast<uint32_t>(FAST_BAUD)) ? DEFAULT_BAUD : FAST_BAUD;
        printf("No answer at %lu baud, trying %lu\n", baud(), static_cast<uint32_t>(other));
        err = sensor.changeHostBaudRate(other);
        return_if_vn_error(err);
        err = sensor.readRegister(&model_register, true);
    }
    return_if_vn_error(err);
    printf("Sensor Model Number: %s\n", model_register.model.c_str());
    printf("baud: %lu\n", baud());
    return VN::Error::None;
}

VN::Error VectorNavIMU::configure() {
    VN::Error err = find_sensor();
    return_if_vn_error(err);

    if (baud() != static_cast<uint32_t>(FAST_BAUD)) {
        err = sensor.changeBaudRate(FAST_BAUD);
        return_if_vn_error(err);
        printf("baud: %lu\n", baud());
    }

    // Fixed: the sensor's default ASCII output (VNYMR at 40Hz) was left on, using link bandwidth
    // that nothing reads
    VN::Registers::System::AsyncOutputType ascii_reg;
    ascii_reg.ador = VN::Registers::System::AsyncOutputType::Ador::OFF;
    ascii_reg.serialPort = VN::Registers::System::AsyncOutputType::SerialPort::ActiveSerial;
    err = sensor.writeRegister(&ascii_reg);
    return_if_vn_error(err);

    err = sensor.writeRegister(&imu_reg);
    return_if_vn_error(err);
    err = sensor.writeRegister(&nav_reg);
    return_if_vn_error(err);

    return VN::Error::None;
}

void VectorNavIMU::disconnect() {
    sensor.disconnect();
    connected = false;
    configured = false;
    link_state = VectornavLink::Disconnected;
}

void VectorNavIMU::record_error(VN::Error err) {
    if (errors < UINT16_MAX) errors++;
    latest_error = err;
}

VN::Error VectorNavIMU::update_state(VectornavState& state) {
    auto now = Kernel::Clock::now();

    if (configured && now - last_packet > DATA_TIMEOUT) {
        printf("No IMU data for %lld ms, reconfiguring\n",
               std::chrono::duration_cast<std::chrono::milliseconds>(now - last_packet).count());
        configured = false;
        link_state = VectornavLink::Configuring;
    }

    if (!configured) {
        if (configure_attempts > 0 && now - last_configure < RETRY_PERIOD) {
            return VN::Error::None;
        }
        return start();
    }

    // Fixed: this read one packet per call (and was called every 10ms), so the sensor's packets
    // piled up in the queue and arrived late or got dropped. switch to draining the queue
    while (VN::Sensor::CompositeDataQueueReturn composite_data = sensor.getNextMeasurement(false)) {
        last_packet = Kernel::Clock::now();
        link_state = VectornavLink::Streaming;

        if (composite_data->matchesMessage(imu_reg)) {
            state.accel = composite_data->imu.accel.value_or(state.accel);
            state.ang_rate = composite_data->imu.angularRate.value_or(state.ang_rate);
        }
        if (composite_data->matchesMessage(nav_reg)) {
            if (composite_data->attitude.ypr.has_value()) {
                state.ypr = wrap_ypr(composite_data->attitude.ypr.value());
            }
            state.pos = composite_data->ins.posLla.value_or(state.pos);
            state.vel = composite_data->ins.velBody.value_or(state.vel);
            state.gnss_fix = composite_data->gnss.gnss1Fix.value_or(state.gnss_fix);
        }
    }

    if (link_state == VectornavLink::Streaming && now - last_packet > 100ms) {
        link_state = VectornavLink::NoData;
    }

    // Handle async errors
    // Fixed: only one error was read per call and its print was commented out, so errors went
    // unseen. switch to draining the error queues.
    VN::Error result = VN::Error::None;
    while (std::optional<VN::AsyncError> async_error = sensor.getNextAsyncError()) {
        printf(
            "Received async error: %s %s\n",
            std::string(VN::errorCodeToString(async_error->error)).c_str(),
            async_error->message.c_str()
        );
        record_error(async_error->error);
        result = async_error->error;
    }

    return result;
}
