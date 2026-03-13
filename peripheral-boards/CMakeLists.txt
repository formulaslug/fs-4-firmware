cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(MBED_APP_JSON_PATH mbed_app.json5)

set(CUSTOM_TARGETS_JSON_PATH ../custom_targets/custom_targets.json5)
set(CUSTOM_UPLOAD_CFG_PATH ../custom_targets/UploadMethods.cmake)

add_subdirectory(../custom_targets custom_targets)

set(MBED_TARGET PERIPHERAL_BOARD)

include(../mbed-os/tools/cmake/mbed_toolchain_setup.cmake)
project(stm32-test
        LANGUAGES C CXX ASM)
include(../mbed-os/tools/cmake/mbed_project_setup.cmake)

add_subdirectory(../mbed-os mbed-os)

add_subdirectory(../libraries/d6t-mems/d6t-1a d6t-1a)
add_subdirectory(../libraries/d6t-mems/d6t-8lh d6t-8lh)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME} 
        main.cpp
        config.cpp
        wheel_speed.cpp
)
target_link_libraries(${PROJECT_NAME} mbed-os d6t-1a d6t-8lh)
mbed_set_post_build(${PROJECT_NAME})
