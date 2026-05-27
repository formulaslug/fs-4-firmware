cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

set(MBED_APP_JSON_PATH mbed_app.json5)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(MBED_APP_JSON_PATH mbed_app.json5)

set(CUSTOM_TARGETS_JSON_PATH ../custom_targets/custom_targets.json5)
set(CUSTOM_UPLOAD_CFG_PATH ../custom_targets/UploadMethods.cmake)

add_subdirectory(../custom_targets custom_targets)

set(MBED_TARGET PERIPHERAL_BOARD) # We never made a custom target for the Charger board...

include(../mbed-os/tools/cmake/mbed_toolchain_setup.cmake)
project(Charger
    LANGUAGES C CXX ASM) 
include(mbed_project_setup)

add_subdirectory(../mbed-os mbed-os)
add_subdirectory(../libraries/bt817q bt817q)

add_executable(${PROJECT_NAME} 
    main.cpp
    dash-layouts/charger_default_layout.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
    .
    dash-layouts
)
target_link_libraries(${PROJECT_NAME} mbed-os bt817q) # Can also link to mbed-baremetal here
mbed_set_post_build(${PROJECT_NAME})
