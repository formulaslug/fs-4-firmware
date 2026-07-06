#include "LTC681xChainBus.h"

#include <cstdint>
#include <cstddef>

#include "LTC681xBus.h"
#include "mbed.h"

template<const unsigned int N_chips>
LTC681xBus::LTC681xBusStatus LTC681xChainBus<N_chips>::WakeupBus() {
  // Pulse the CS once per chip, delaying the T_wake time between
  for(uint8_t i = 0; i < N_chips; i++) {
    m_spiDriver->select();
    m_spiDriver->deselect();
    wait_us(LTC681x_WAKEUP_DELAY);
  }

  return LTC681xBus::LTC681xBusStatus::Ok;
}

template<const unsigned int N_chips>
LTC681xBus::LTC681xBusStatus LTC681xChainBus<N_chips>::SendCommand(LTC681xBus::BusCommand cmd) {
  //
  // In daisy chain mode, to send a command we broadcast
  // a single command to all devices.
  //

  // Be sure we are trying to send a chain command
  MBED_ASSERT(cmd.mode == AddressingMode::kChain);

  uint8_t cmdBytes[4];
  LTC681xBus::getCommandBytes(cmdBytes, cmd);

  // Grab the bus and send our command
  m_spiDriver->select();
  m_spiDriver->write((const char*)cmdBytes, sizeof(cmdBytes), NULL, 0);
  m_spiDriver->deselect();

  return LTC681xBus::LTC681xBusStatus::Ok;
}

template<const unsigned int N_chips>
LTC681xBus::LTC681xBusStatus LTC681xChainBus<N_chips>::SendDataCommand(LTC681xBus::BusCommand cmd, uint8_t* data) {
  //
  // In daisy chain mode, to send a command with data, we first
  // broadcast the command code once to all devices, then send
  // data for the devices in order.
  //

  // Be sure we are trying to send a chain command
  MBED_ASSERT(cmd.mode == AddressingMode::kChain);

  // Create command value array
  uint8_t cmdBytes[4];
  LTC681xBus::getCommandBytes(cmdBytes, cmd);

  // Create data value array
  uint8_t dataBytes[N_chips * 8];
  for (uint8_t chip = 0; chip < N_chips; chip++) {
      LTC681xBus::getDataBytes(dataBytes + (chip * 8), data + (chip * 6));
  }

  // Grab the bus and send our command
  m_spiDriver->select();
  m_spiDriver->write((const char*)cmdBytes, 4, NULL, 0);
  m_spiDriver->write((const char*)dataBytes, N_chips * 8, NULL, 0);
  m_spiDriver->deselect();

  return LTC681xBus::LTC681xBusStatus::Ok;
}

template<const unsigned int N_chips>
LTC681xBus::LTC681xBusStatus LTC681xChainBus<N_chips>::SendReadCommand(LTC681xBus::BusCommand cmd, uint8_t* data) {
  //
  // In daisy chain mode, to send a read command, we first send the
  // command code, then read in 8 bytes from each chip in the chain.
  //

  // Be sure we are trying to send a chain command
  MBED_ASSERT(cmd.mode == AddressingMode::kChain);

  // Create command value array
  uint8_t cmdBytes[4];
  LTC681xBus::getCommandBytes(cmdBytes, cmd);

  // Create received values array
  uint8_t rxbuf[N_chips * 8];

  // Grab the bus and send our command
  m_spiDriver->select();
  m_spiDriver->write((const char*)cmdBytes, 4, NULL, 0);
  m_spiDriver->write(NULL, 0, (char*)rxbuf, N_chips * 8);
  m_spiDriver->deselect();

  // Calculate PEC and copy data over
  for(size_t chip = 0; chip < N_chips; chip++) {
    uint8_t* chipBuf = rxbuf + (chip * 8);

    // Check that the pec we got lines up with the computed
    uint16_t dataPec = LTC681xBus::calculatePec(6, chipBuf);
    if((dataPec >> 8) != chipBuf[6] || (dataPec & 0xff) != chipBuf[7]) {
      // If not, return error
      return LTC681xBus::LTC681xBusStatus::BadPec;
    }

    // Copy the good data over
    for(size_t i = 0; i < 6; i++) {
      data[(chip * 6) + i] = chipBuf[i];
    }
  }

  return LTC681xBus::LTC681xBusStatus::Ok;
}

template <const unsigned int N_chips>
LTC681xBus::LTC681xBusStatus LTC681xChainBus<N_chips>::SendCommandAndPoll(BusCommand cmd, unsigned int timeout) {
  //
  // In daisy chain mode, to send a command we broadcast
  // a single command to all devices.
  //

  // Be sure we are trying to send a chain command
  MBED_ASSERT(cmd.mode == AddressingMode::kChain);

  uint8_t cmdBytes[4];
  LTC681xBus::getCommandBytes(cmdBytes, cmd);

  // Grab the bus and send our command
  m_spiDriver->select();
  m_spiDriver->write((const char*)cmdBytes, sizeof(cmdBytes), NULL, 0);

  // Poll for ADC completion

  // Send initial N isoSPI clock pulses
  for(unsigned int i = 0; i < N_chips / 8; i++) {
    m_spiDriver->write(0xff);
  }

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

template<const unsigned int N_chips>
LTC681xBus::LTC681xBusStatus LTC681xChainBus<N_chips>::PollAdcCompletion(BusCommand cmd, unsigned int timeout) {
  //
  // In daisy chain mode, to send a command we broadcast
  // a single command to all devices.
  //

  // Be sure we are trying to send a chain command
  MBED_ASSERT(cmd.mode == AddressingMode::kChain);
  // Be sure we are sending a PLADC command
  MBED_ASSERT(cmd.command == 0x0714);

  uint8_t cmdBytes[4];
  LTC681xBus::getCommandBytes(cmdBytes, cmd);

  // Grab the bus and send our command
  m_spiDriver->select();
  m_spiDriver->write((const char*)cmdBytes, sizeof(cmdBytes), NULL, 0);

  // Poll for ADC completion

  // Send initial N isoSPI clock pulses
  for(unsigned int i = 0; i < N_chips / 8; i++) {
    m_spiDriver->write(0xff);
  }

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
