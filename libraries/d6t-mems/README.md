# FS D6T-8LH & D6T-1A Sensor Library

A CMake library for interfacing with the Omron D6T-8LH and D6T-1A thermal infrared sensors over I2C, extracted from FS-3 telemetry.

Roughly based on:  
https://github.com/Voltage-Ranger-Electronics/D6T-Temperature-Sensor/blob/master/Arduino%20D6T_8L_06_Demo/D6T_8L_06_Demo.ino

D6T Series User Manual datasheet:
https://www.digikey.es/htmldatasheets/production/3713333/0/0/1/d6t8l06.html

---

## Library Structure
- `D6T8LH` — driver for the Omron D6T-8LH (8-pixel array)
- `D6T1A` — driver for the Omron D6T-1A (single-pixel)

Each sensor is implemented as a C++ class that owns its I2C reference and internal buffers.

---

## Usage

### CMake
Add the libraries as subdirectories and link them:

```cmake
add_subdirectory(path/to/d6t-8lh d6t-8lh)
target_link_libraries(your_target d6t-8lh)

add_subdirectory(path/to/d6t-1a d6t-1a)
target_link_libraries(your_target d6t-1a)
```
