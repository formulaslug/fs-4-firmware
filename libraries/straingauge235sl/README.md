# FS 235SL Strain Gauge Library

A standalone CMake library for interfacing with a Micro-Measurements 235SL strain gauge using a quarter-bridge configuration and INA125U instrumentation amplifier.

---

## Overview

This library reads the analog output of the INA125U amplifier and provides:
- Raw  ADC readings (0.0 – 1.0)
- Filtered voltage readings (0 – Vref)
- Calibrated physical units (force/strain)

The conversion model used is: strain = (voltage - tare_voltage) * slope + intercept 

The driver implements:
- Moving average filtering
- Tare
- Linear calibration
- Two-point calibration helper 
- Saturation diagnostics 

---

## Dependencies

- Mbed OS
- ARM GCC toolchain
- STM32 MCU with ADC input

---

## Usage
Add this as a subdirectory in your CMake project:

```cmake
add_subdirectory(path/to/fs-strain-gauge fs_strain_gauge)
target_link_libraries(your_target fs_strain_gauge)
```