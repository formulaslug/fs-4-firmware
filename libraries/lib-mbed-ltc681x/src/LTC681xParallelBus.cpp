#include "LTC681xParallelBus.h"

#include "mbed.h"

LTC681xBus::LTC681xBusStatus LTC681xParallelBus::WakeupBus() {
  // Pulse the CS and delay for T_wake
  m_spiDriver->select();
  m_spiDriver->deselect();
  wait_us(LTC681x_WAKEUP_DELAY);

  return LTC681xBus::LTC681xBusStatus::Ok;
}

LTC681xBus::LTC681xBusStatus LTC681xParallelBus::SendCommand(LTC681xBus::BusCommand cmd) {
  uint8_t cmdBytes[4];
  LTC681xBus::getCommandBytes(cmdBytes, cmd);

  // Grab the bus and send our command
  m_spiDriver->select();
  m_spiDriver->write((const char*)cmdBytes, sizeof(cmdBytes), NULL, 0);
  m_spiDriver->deselect();

  return LTC681xBus::LTC681xBusStatus::Ok;
}

LTC681xBus::LTC681xBusStatus LTC681xParallelBus::SendDataCommand(LTC681xBus::BusCommand cmd, uint8_t* data) {
  // Create command value array
  uint8_t cmdBytes[4];
  LTC681xBus::getCommandBytes(cmdBytes, cmd);

  // Create data value array
  uint8_t dataBytes[8];
  LTC681xBus::getDataBytes(dataBytes, data);

  // Grab the bus and send our command
  m_spiDriver->select();
  m_spiDriver->write((const char*)cmdBytes, 4, NULL, 0);
  m_spiDriver->write((const char*)dataBytes, 8, NULL, 0);
  m_spiDriver->deselect();

  return LTC681xBus::LTC681xBusStatus::Ok;
}

LTC681xBus::LTC681xBusStatus LTC681xParallelBus::SendReadCommand(LTC681xBus::BusCommand cmd, uint8_t* data) {
  // Create command value array
  uint8_t cmdBytes[4];
  LTC681xBus::getCommandBytes(cmdBytes, cmd);

  // Create received values array
  uint8_t dataBytes[8];

  // Grab the bus and send our command
  m_spiDriver->select();
  m_spiDriver->write((const char*)cmdBytes, 4, NULL, 0);
  m_spiDriver->write(NULL, 0, (char*)dataBytes, 8);
  m_spiDriver->deselect();

  // Check that the pec we got lines up with the computed
  uint16_t dataPec = LTC681xBus::calculatePec(6, dataBytes);
  if((dataPec >> 8) != dataBytes[6] || (dataPec & 0xff) != dataBytes[7]) {
    // If not, return error
    return LTC681xBus::LTC681xBusStatus::BadPec;
  }

  // Copy the good data over
  for(size_t i = 0; i < 6; i++) {
    data[i] = dataBytes[i];
  }

  return LTC681xBus::LTC681xBusStatus::Ok;
}

LTC681xBus::LTC681xBusStatus LTC681xParallelBus::SendCommandAndPoll(LTC681xBus::BusCommand cmd, unsigned int timeout) {
  // Be sure we are trying to send a chain command
  MBED_ASSERT(cmd.mode == AddressingMode::kAddress);

  uint8_t cmdBytes[4];
  LTC681xBus::getCommandBytes(cmdBytes, cmd);

  // Grab the bus and send our command
  m_spiDriver->select();
  m_spiDriver->write((const char*)cmdBytes, sizeof(cmdBytes), NULL, 0);

  // Send clock pulses until devices are not busy (non-zero value)
  bool gotResponse = false;
  for(unsigned int cycle = 0; cycle < (timeout * 1000 / LTC681x_POLL_DELAY); cycle++) {
    uint8_t readVal;
    m_spiDriver->write(NULL, 0, (char*) &readVal, 1);
    if(readVal != 0) {
      gotResponse = true;
      break;
    } else {
      wait_us(LTC681x_POLL_DELAY);
    }
  }
  m_spiDriver->deselect();

  if(gotResponse) {
    return LTC681xBus::LTC681xBusStatus::Ok;
  } else {
    return LTC681xBus::LTC681xBusStatus::PollTimeout;
  }
}

LTC681xBus::LTC681xBusStatus LTC681xParallelBus::PollAdcCompletion(LTC681xBus::BusCommand cmd, unsigned int timeout) {
  // Be sure we are trying to send a chain command
  MBED_ASSERT(cmd.mode == AddressingMode::kAddress);
  // Be sure we are sending a PLADC command
  MBED_ASSERT(cmd.command == 0x0714);

  uint8_t cmdBytes[4];
  LTC681xBus::getCommandBytes(cmdBytes, cmd);

  // Grab the bus and send our command
  m_spiDriver->select();
  m_spiDriver->write((const char*)cmdBytes, sizeof(cmdBytes), NULL, 0);

  // Send clock pulses until devices are not busy (non-zero value)
  bool gotResponse = false;
  for(unsigned int cycle = 0; cycle < (timeout * 1000 / LTC681x_POLL_DELAY); cycle++) {
    uint8_t readVal;
    m_spiDriver->write(NULL, 0, (char*) &readVal, 1);
    if(readVal != 0) {
      gotResponse = true;
      break;
    } else {
      wait_us(LTC681x_POLL_DELAY);
    }
  }  
  m_spiDriver->deselect();

  if(gotResponse) {
    return LTC681xBus::LTC681xBusStatus::Ok;
  } else {
    return LTC681xBus::LTC681xBusStatus::PollTimeout;
  }
}
