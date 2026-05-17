/******************************************************************************
 * File    : BnoBus.h
 * Module  : 7Semi BNO08x — Transport Abstraction
 * Version : 0.1.0
 * SPDX-License-Identifier: MIT
 *
 * Minimal transport interface that unifies I2C / SPI / UART behind a single API
 * for the BNO08x SHTP protocol. Concrete buses (e.g., BnoI2CBus, BnoSPIBus,
 * BnoUARTBus) implement this interface and are injected into the driver.
 ******************************************************************************/

#pragma once
#include "mbed.h"

/**
 * BNO08x transport interface
 * - Unifies I2C / SPI / UART behind a single API.
 */
struct BnoBus {
  virtual ~BnoBus() {}

  /**
   * Initialize the peripheral.
   * - Configure pins/clock/mode as appropriate for the concrete bus.
   * - Return true when the bus is ready to use.
   */
  virtual bool begin() = 0;

  /**
   * Transmit a complete SHTP frame.
   * - data : pointer to contiguous bytes (SHTP header + payload)
   * - n    : number of bytes to send
   * - return : true when all bytes were handed to the peripheral successfully
   */
  virtual bool tx(const uint8_t* data, size_t n) = 0;

  /**
   * Receive one complete SHTP frame.
   * - buf  : destination buffer (caller-provided)
   * - cap  : capacity of 'buf' (must be >= 4 for SHTP header)
   * - len  : OUT — full frame length (including 4-byte SHTP header)
   * - return : number of bytes actually copied into buf (0 on timeout/error)
   */
  virtual int rx(uint8_t* buf, size_t cap) = 0;
};

