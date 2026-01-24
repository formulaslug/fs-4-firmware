# FS BT817Q Dash Library

A standalone CMake library for controlling the BT817Q LCD display controller, extracted from the FS-3 project.

## Dependencies
- Mbed OS
- ARM GCC toolchain

## Usage
Add this as a subdirectory in your CMake project:

```cmake
add_subdirectory(path/to/fs-bt817q-dash bt817q-dash)
target_link_libraries(your_target bt817q-dash)
```

## API
See BT817Q.hpp for the main interface.

## Building
Requires CMake 3.19+ and Mbed OS setup.