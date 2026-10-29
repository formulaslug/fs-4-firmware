cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(MBED_APP_JSON_PATH mbed_app.json5)

set(MBED_TARGET NUCLEO_F446RE)
set(UPLOAD_METHOD OPENOCD CACHE STRING "Method for uploading programs to the mbed")

include(../mbed-os/tools/cmake/mbed_toolchain_setup.cmake)
project(imu-board
        LANGUAGES C CXX ASM)
include(../mbed-os/tools/cmake/mbed_project_setup.cmake)

add_subdirectory(../mbed-os mbed-os)
add_subdirectory(../libraries/vectornav-sdk-1-2-0-mbed vectornav-sdk)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME}
        main.cpp
        vectornav_imu.cpp
)

target_link_libraries(${PROJECT_NAME} mbed-os oVnSensor)
mbed_set_post_build(${PROJECT_NAME})
