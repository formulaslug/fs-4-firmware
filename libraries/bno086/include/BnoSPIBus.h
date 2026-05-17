/******************************************************************************
 * File    : BnoSPIBus.h
 * Module  : 7Semi BNO08x — SPI Transport Adapter
 * Version : 0.1.0
 * License : MIT
 *
 * Summary
 * -------
 * SPI transport for the BNO08x SHTP protocol. Presents the uniform BnoBus
 * interface  so the higher-level driver stays transport-agnostic.
 * Configuration options allow INT and RESET pins, SPI clock/mode,
 * and ESP32/ESP8266 custom SPI pin mapping.
 ******************************************************************************/

#pragma once

#include "BnoBus.h"
#include "mbed.h"

/** ---------------------------------------------------------------------------
 *  BNO08x SPI transport configuration
 *
 *  - Uses standard 4-byte SHTP header
 *  - SPI_MODE0 and MSB-first expected by BNO08x
 *  - Default clock is 1 MHz (safe for most boards)
 *  ------------------------------------------------------------------------- */


/** ---------------------------------------------------------------------------
 *  Globals (single-device HAL)
 *
 *  - Used by the ISR and bus instance as a shared HAL layer
 *  - Assumes only a single BNO08x device is active on SPI
 *  - Allows static ISR to signal instance logic using g_intFlag
 *  ------------------------------------------------------------------------- */
// static SPISettings g_settings(1000000, MSBFIRST, SPI_MODE0);

/** ---------------------------------------------------------------------------
 *  BnoSPIBus
 *
 *  - Implements the generic BnoBus transport interface
 *  - Provides SPI begin(), tx(), and rx() primitives for SHTP
 *  - Supports optional INT and RESET pins
 *  - Supports custom SPI pin mapping
 *  ------------------------------------------------------------------------- */
struct BnoSPIBus : public BnoBus {
  SPI spi;
  DigitalOut cs;
  DigitalIn intn;
  DigitalOut rst;
  DigitalOut wake;
  uint32_t clk;
  uint8_t mode;
  PinName sck, miso, mosi;

  /** -------------------------------------------------------------------------
   *  Constructor
   *
   *  - Allows SPI object override (SPI, HSPI, VSPI, etc.)
   *  - Allows optional INT, RESET, and WAKE pins
   *  - Allows setting SPI clock + mode
   *  ----------------------------------------------------------------------- */
  BnoSPIBus(PinName csPin = NC,
            PinName intnPin = NC,
            PinName rstPin = NC,
            PinName wakePin = NC,
            uint32_t clkIn = 1000000,
            uint8_t spiMode = 3,
            PinName sckPin = NC,
            PinName misoPin = NC,
            PinName mosiPin = NC
          )
    : spi(mosiPin, misoPin, sckPin),
      cs(csPin),
      intn(intnPin),
      rst(rstPin, 1),
      wake(wakePin, 1),
      clk(clkIn),
      mode(spiMode),
      sck(sckPin),
      miso(misoPin),
      mosi(mosiPin) {}

  /** -------------------------------------------------------------------------
   *  begin()
   *
   *  - Initializes SPI peripheral and chip select pin
   *  - Configures INT pin interrupt if provided
   *  - Performs optional reset pulse sequence for BNO08x
   *  - Stores configuration into global variables
   *
   *  Notes:
   *  - Returns false if SPI object is invalid or CS pin is not set
   *  - INT pin is configured using INPUT_PULLUP and FALLING edge trigger
   *  ----------------------------------------------------------------------- */
  bool begin() override {
    // g_settings = SPISettings(clk, MSBFIRST, mode);
    cs = 1;
    spi.format(8,mode);
    spi.frequency(clk);

    /** CS pin
     *
     *  - Active LOW
     *  - Default idle state HIGH
     */
    cs = 0;

    /** INT pin
     *
     *  - Optional
     *  - Uses FALLING edge to signal FIFO has data
     */
    intn.mode(PullUp);

    /** Reset sequence (recommended for BNO08x)
     *
     *  - Optional
     *  - Pulse LOW then allow device boot time
     */
    if (true) {
      rst = 1;
      ThisThread::sleep_for(5ms); 
      rst = 0;
      ThisThread::sleep_for(10ms); 
      rst = 1;
      ThisThread::sleep_for(300ms); 
    }
    cs = 1;
    return true;
  }

  /** -------------------------------------------------------------------------
   *  tx()
   *
   *  - Writes a raw SHTP packet to the device
   *  - Uses SPI transactions for safe multi-device bus usage
   *  - Transfers bytes using platform-appropriate method
   *
   *  Notes:
   *  - ESP32 supports writeBytes() for fast DMA-style write
   *  - Other MCUs use transfer() loop for compatibility
   *  ----------------------------------------------------------------------- */
  bool tx(const uint8_t *data, size_t n) override {
    if (!data || n == 0) return false;

    // Wake up the BNO08x (required by SHTP over SPI spec)
    wake = 0;
    int timeout = 0;
    while (intn) { // wait for INTN to go low indicating BNO is ready
      timeout++;
      if (timeout > 100) break; // Timeout
      ThisThread::sleep_for(1ms);
    }
    printf("Begin transmission\n");

    // spi->beginTransaction(g_settings);
    cs = 0;
    wait_us(5);

    for (size_t i = 0; i < n; i++) spi.write(data[i]);
    //  for (size_t i = 0; i < n; i++)
    // spi.write(data[i]);
      
    wait_us(5);
    cs = 1;
    wake = 1; // Release WAKE pin
    // spi->endTransaction();

    return true;
  }

  /** -------------------------------------------------------------------------
   *  rx()
   *
   *  - Reads one or more packets from the BNO08x FIFO
   *  - Handles continuation packets (MSB of length set)
   *  - Copies the raw SHTP packet stream into caller buffer
   *
   *  Inputs:
   *  - buf    : output buffer to store packet(s)
   *  - cap    : buffer size (max allowed storage)
   *  - len    : populated with number of valid bytes received
   *
   *  Output:
   *  - Returns number of bytes read (0 if nothing / failure)
   *
   *  Notes:
   *  - INT-based gating is intentionally bypassable for debug
   *  - Each packet includes its own 4-byte header
   *  - Stops if packet size invalid or buffer would overflow
   *  ----------------------------------------------------------------------- */
  int rx(uint8_t *buf, size_t cap) override {
    uint16_t len = 0;
    /** Optional INT gating
     *
     *  - When enabled, rx() will only read after INT asserts
     *  - Can be bypassed for debug sessions
     */
    if (false) { // optional INT pin
      int i = 0;
      while (intn) { // wait for INT to go LOW
        printf("Wait intn\n");
        i++;
        if (i > 100) // Timeout after ~100 ms
          return 0;  // nothing to read
        ThisThread::sleep_for(1ms);
      }
    }

    printf("Begin recv\n");
    while (true) {
      uint8_t hdr[4]; // SHTP header buffer

      cs = 0; // select device

      /** Read 4-byte SHTP header
       *
       *  - First 2 bytes: packet length (with continuation flag)
       *  - Next 2 bytes : channel + sequence
       */
      for (int i = 0; i < 4; i++)
        hdr[i] = spi.write(0x00); // dummy write to clock data out
      /** Parse packet length and continuation flag */
      uint16_t pktLen = hdr[0] | (hdr[1] << 8);
      bool ok = pktLen & 0x8000;  // MSB = ok to continue
      pktLen &= ~0x8000;              // clear MSB
      /** Packet sanity check
       *
       *  - pktLen must at least include header
       *  - total must fit in user buffer
       */
      if (pktLen < 4 || pktLen > cap) {
        cs = 1;
        // spi->endTransaction();
        break;
      }

      /** Copy header into buffer */
      memcpy(buf + len, hdr, 4);

      /** Read payload bytes
       *
       *  - Payload length = pktLen - headerLen
       *  - Uses dummy reads (0xFF) to clock data out
       */
      uint16_t payloadLen = pktLen - 4; // exclude header
      //  Read payload bytes
      for (uint16_t i = 0; i < payloadLen; i++) {
        buf[len + 4 + i] = spi.write(0xFF);// dummy write to clock data out
      }

      cs = 1; // deselect device
      // spi->endTransaction();// end SPI transaction
      len += pktLen;// update total length
      /** Exit when FIFO drained */
      if (!ok) break;
    }
    ThisThread::sleep_for(3ms);
    /** Clear interrupt flag after draining FIFO */
    return len;
  }
};
