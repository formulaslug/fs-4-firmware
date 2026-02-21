#pragma once

#include <cstdint>

#include "LTC681xCommand.h"

// Delays in us
#define LTC681x_WAKEUP_DELAY 420
#define LTC681x_POLL_DELAY 10

class LTC681xBus {
public:
  enum class AddressingMode : uint8_t {
    kBroadcast = 0x00,
    kChain = 0x00,
    kAddress = 0x01
  };
  typedef uint8_t CommandAddress;
  typedef uint16_t CommandCode;

  union BusCommand {
    struct {
      CommandCode command : 11;
      CommandAddress address : 4;
      AddressingMode mode : 1;
    };
    uint16_t value;
  };

  static BusCommand BuildAddressedBusCommand(const LTC681xCommand& command, CommandAddress address);
  static BusCommand BuildBroadcastBusCommand(const LTC681xCommand& command);
  static BusCommand BuildChainBusCommand(const LTC681xCommand& command);


  enum class LTC681xBusStatus : uint8_t {
    Ok,
    BadPec,
    PollTimeout
  };

  virtual LTC681xBusStatus WakeupBus() = 0;

  virtual LTC681xBusStatus SendCommand(BusCommand cmd) = 0;
  virtual LTC681xBusStatus SendDataCommand(BusCommand cmd, uint8_t* data) = 0;
  virtual LTC681xBusStatus SendReadCommand(BusCommand cmd, uint8_t* data) = 0;

  virtual LTC681xBusStatus SendCommandAndPoll(BusCommand cmd, unsigned int timeout = 10) = 0;
  virtual LTC681xBusStatus PollAdcCompletion(BusCommand cmd, unsigned int timeout = 10) = 0;

protected:
  static uint16_t calculatePec(uint8_t length, uint8_t *data);
  static void getCommandBytes(uint8_t* buf, BusCommand cmd);
  static void getDataBytes(uint8_t* buf, uint8_t data[6]);
};
