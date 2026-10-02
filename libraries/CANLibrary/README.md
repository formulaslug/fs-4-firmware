# CANLibrary

Auto-generates C++ structs with `encode()` / `decode()` methods from `CANbus.dbc`.

## Prerequisites

- `dbcc` — DBC-to-C compiler (must be on `PATH`)
- `python3`

## Adding to Your Project

In your project's `CMakeLists.txt`:

```cmake
add_subdirectory(../libraries/CANLibrary CANLibrary)
target_link_libraries(${PROJECT_NAME} mbed-os CANLibrary)
```

## Usage

```cpp
#include "CanGenerated.h"

// Encode a message
struct ACC_TPDO_STATUS msg;
msg.ACC_STATUS_BMS_FAULT = true;
msg.ACC_STATUS_SOC = 85;
CANMessage can_msg = msg.encode();
canBus.write(can_msg);

// Decode a message
struct ACC_TPDO_STATUS received;
if (received.decode(incoming_msg)) {
    printf("BMS Fault: %d\n", received.ACC_STATUS_BMS_FAULT);
}
```

## How It Works

1. **CMake runs `dbcc`** on `CANbus.dbc` → generates `CANbus.c` and `CANbus.h` (low-level pack/unpack functions).
2. **CMake runs `generate_can_helpers.py`** → reads `CANbus.h` and generates `CanGenerated.h` with one struct per CAN message containing:
   - Fields initialized to `0`
   - `encode()` — packs fields into a `CANMessage`
   - `decode()` — unpacks a `CANMessage` into fields (uses `memcpy` for alignment safety)

## Updating the DBC

Replace `CANbus.dbc` with your updated file. The library will regenerate automatically on the next build.
