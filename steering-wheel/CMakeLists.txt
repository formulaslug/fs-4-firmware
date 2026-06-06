cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(MBED_APP_JSON_PATH mbed_app.json5)

set(CUSTOM_TARGETS_JSON_PATH ../custom_targets/custom_targets.json5)
set(CUSTOM_UPLOAD_CFG_PATH ../custom_targets/UploadMethods.cmake)

add_subdirectory(../custom_targets custom_targets)

set(MBED_TARGET STEERING_WHEEL_BOARD)
        
include(../mbed-os/tools/cmake/mbed_toolchain_setup.cmake)
project(steering-wheel
        LANGUAGES C CXX ASM)
include(../mbed-os/tools/cmake/mbed_project_setup.cmake)

add_subdirectory(../mbed-os mbed-os)
add_subdirectory(../libraries/bt817q bt817q)
add_subdirectory(../libraries/FilterUtils debounced_digital_in)

set(CMAKE_CXX_STANDARD 20)

# Auto-generate CANbus.c / CANbus.h from CANbus.dbc using dbcc by Howerj
# Generated output files placed in build directory
# Was building off the add-can-dbc library
set(DBC_FILE     ${CMAKE_CURRENT_SOURCE_DIR}/../CANbus.dbc)
set(GENERATED_C  ${CMAKE_CURRENT_BINARY_DIR}/CANbus.c)
set(GENERATED_H  ${CMAKE_CURRENT_BINARY_DIR}/CANbus.h)

add_custom_command(
    OUTPUT  ${GENERATED_C} ${GENERATED_H}
    COMMAND dbcc ${DBC_FILE}
    DEPENDS ${DBC_FILE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating CANbus.c and CANbus.h from CANbus.dbc"
)

add_executable(${PROJECT_NAME}
        main.cpp
        dash_screen.cpp
        ${GENERATED_C}  
)
add_custom_target(generate_canbus DEPENDS ${GENERATED_C} ${GENERATED_H})
add_dependencies(${PROJECT_NAME} generate_canbus)

target_include_directories(${PROJECT_NAME} PUBLIC . ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(${PROJECT_NAME} mbed-os bt817q filter_utils)
mbed_set_post_build(${PROJECT_NAME})
