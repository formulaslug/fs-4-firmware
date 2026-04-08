cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(MBED_APP_JSON_PATH mbed_app.json5)

set(CUSTOM_TARGETS_JSON_PATH ../custom_targets/custom_targets.json5)
set(CUSTOM_UPLOAD_CFG_PATH ../custom_targets/UploadMethods.cmake)

add_subdirectory(../custom_targets custom_targets)

set(MBED_TARGET VEHICLE_CONTROL_UNIT)

include(../mbed-os/tools/cmake/mbed_toolchain_setup.cmake)
project(vcu
        LANGUAGES C CXX ASM)
include(../mbed-os/tools/cmake/mbed_project_setup.cmake)

add_subdirectory(../mbed-os mbed-os)
# add_subdirectory(../libraries/FilterUtils FilterUtils)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME} 
        main.cpp
        ./etc/etc_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ./etc)
target_link_libraries(${PROJECT_NAME} mbed-os)

mbed_set_post_build(${PROJECT_NAME})
