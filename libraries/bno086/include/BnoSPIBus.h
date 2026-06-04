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
 *  - SPI mode 3 and MSB-first expected by BNO08x
 *  - Default clock is 1 MHz (safe for most boards)
 *  ------------------------------------------------------------------------- */


/** ---------------------------------------------------------------------------
 *  Globals (single-device HAL)
 *
 *  - Used by the ISR and bus instance as a shared HAL layer
 *  - Assumes only a single BNO08x device is active on SPI
 *  - Allows static ISR to signal instance logic using g_intFlag
 *  ------------------------------------------------------------------------- */
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
      cs(csPin, 1),
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
    cs = 1;
    spi.format(8,mode);
    spi.frequency(clk);

    /** INT pin
     *
     *  - Optional
     *  - Uses FALLING edge to signal FIFO has data
     */
    if (intn.is_connected()) {
      intn.mode(PullUp);
    }

    /** Reset sequence (recommended for BNO08x)
     *
     *  - Optional
     *  - Pulse LOW then allow device boot time
     */
    if (true) {
      wake = 1;
      rst = 1;
      ThisThread::sleep_for(std::chrono::milliseconds(5));
      rst = 0;
      ThisThread::sleep_for(std::chrono::milliseconds(10));
      rst = 1;
      ThisThread::sleep_for(std::chrono::milliseconds(300));
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
    if (intn.is_connected()) {
      int timeout = 0;
      while (intn) { // wait for INTN to go low indicating BNO is ready
        if (++timeout > 125) {
          wake = 1;
          return false;
        }
        ThisThread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    cs = 0;
    wait_us(5);

    for (size_t i = 0; i < n; i++) spi.write(data[i]);
      
    wait_us(5);
    cs = 1;
    wake = 1;

    return true;
  }

  /** -------------------------------------------------------------------------
   *  rx()
   *
   *  - Reads one packet from the BNO08x FIFO
   *  - Clears the continuation flag from the length field
   *  - Copies the raw SHTP packet into caller buffer
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
   *  - INT gates reads when connected; invalid headers are rejected
   *  - Each packet includes its own 4-byte header
   *  - Rejects oversized packets so a bad header cannot stall the app
   *  ----------------------------------------------------------------------- */
  int rx(uint8_t *buf, size_t cap) override {
    if (!buf || cap < 4) return 0;

    if (intn.is_connected() && intn) {
      return 0;
    }

    uint8_t hdr[4]; // SHTP header buffer

    cs = 0; // select device
    wait_us(5);

    /** Read 4-byte SHTP header
     *
     *  - First 2 bytes: packet length (with continuation flag)
     *  - Next 2 bytes : channel + sequence
     */
    for (int i = 0; i < 4; i++)
      hdr[i] = spi.write(0x00); // dummy write to clock data out

    /** Parse packet length and continuation flag */
    uint16_t pktLen = hdr[0] | (hdr[1] << 8);
    pktLen &= ~0x8000; // clear continuation flag
    uint8_t channel = hdr[2] & 0x0F;

    /** Packet sanity check
     *
     *  - pktLen must at least include header
     *  - pktLen must fit the caller's buffer
     */
    if ((hdr[0] == 0xFF && hdr[1] == 0xFF) || pktLen < 4 || channel > 5) {
      cs = 1;
      return 0;
    }

    uint16_t payloadLen = pktLen - 4; // exclude header
    if (pktLen > cap) {
      cs = 1;
      return 0;
    }

    /** Copy header into buffer */
    memcpy(buf, hdr, 4);

    /** Read payload bytes
     *
     *  - Payload length = pktLen - headerLen
     *  - Uses dummy reads (0xFF) to clock data out
     */
    for (uint16_t i = 0; i < payloadLen; i++) {
      buf[4 + i] = spi.write(0xFF);// dummy write to clock data out
    }

    cs = 1; // deselect device
    return pktLen;
  }
};
