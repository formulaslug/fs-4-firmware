# FS DS18B20 Sensor Library

A standalone CMake library for interfacing with DS18B20 temperature sensors via OneWire, extracted from the FS-3 BMS project.

## Dependencies
- Mbed OS
- OneWire library (mbed-onewire)
- ARM GCC toolchain

## Usage
Add this as a subdirectory in your CMake project:

```cmake
add_subdirectory(path/to/fs-ds18b20-sensor ds18b20)
target_link_libraries(your_target ds18b20)
```

## API
See DS18B20.h for the class interface.

## Building
Requires CMake 3.19+ and Mbed OS setup.