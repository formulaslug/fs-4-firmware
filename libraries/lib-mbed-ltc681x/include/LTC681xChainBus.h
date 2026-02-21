#pragma once

#include <cstdint>
#include "mbed.h"

#include "LTC681xBus.h"

template<const unsigned int N_chips>
class LTC681xChainBus : public LTC681xBus {
public:
  LTC681xChainBus(SPI* spiDriver) : m_spiDriver(spiDriver) {};

  LTC681xBusStatus WakeupBus();

  LTC681xBusStatus SendCommand(BusCommand cmd);
  LTC681xBusStatus SendDataCommand(BusCommand cmd, uint8_t* data);
  LTC681xBusStatus SendReadCommand(BusCommand cmd, uint8_t* data);

  LTC681xBusStatus SendCommandAndPoll(BusCommand cmd, unsigned int timeout = 10);
  LTC681xBusStatus PollAdcCompletion(BusCommand cmd, unsigned int timeout = 10);
private:
  SPI* m_spiDriver;
};
