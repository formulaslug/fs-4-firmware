cmake_minimum_required(VERSION 3.19)
project(CANLibrary VERSION 1.0.0)

# Paths
set(DBC_FILE ${CMAKE_CURRENT_SOURCE_DIR}/CANbus.dbc)
set(GENERATOR_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/generate_can_helpers.py)

# Generated output files (placed in the build directory)
set(GENERATED_HEADER ${CMAKE_CURRENT_BINARY_DIR}/CanGenerated.h)
set(GENERATED_C ${CMAKE_CURRENT_BINARY_DIR}/CANbus.c)
set(GENERATED_H ${CMAKE_CURRENT_BINARY_DIR}/CANbus.h)

# Custom command: run dbcc and the Python generator
add_custom_command(
    OUTPUT ${GENERATED_HEADER} ${GENERATED_C} ${GENERATED_H}
    COMMAND dbcc ${DBC_FILE}
    COMMAND python3 ${GENERATOR_SCRIPT} ${GENERATED_H} ${GENERATED_HEADER}
    DEPENDS ${DBC_FILE} ${GENERATOR_SCRIPT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating CANLibrary files from DBC"
)

add_custom_target(CANGen DEPENDS ${GENERATED_HEADER} ${GENERATED_C} ${GENERATED_H})

# Static library from generated C source
add_library(CANLibrary STATIC ${GENERATED_C})
target_include_directories(CANLibrary PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(CANLibrary mbed-core-flags)
add_dependencies(CANLibrary CANGen)
