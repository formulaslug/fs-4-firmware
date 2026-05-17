#pragma once

/**
 * Compile-time bus selection
 * - Define exactly one: BNO_USE_I2C / BNO_USE_SPI / BNO_USE_UART
 * - Includes only the chosen transport
 */
#if !defined(BNO_USE_I2C) && !defined(BNO_USE_SPI) && !defined(BNO_USE_UART)
  #define BNO_USE_SPI
#endif

#ifdef BNO_USE_SPI
  #include "BnoSPIBus.h"
  using BnoSelectedBus = BnoSPIBus;
#endif