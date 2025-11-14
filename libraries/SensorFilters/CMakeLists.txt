cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19)

set(CMAKE_CXX_STANDARD 17)

project(sensor_filters LANGUAGES CXX)

add_definitions(
        -DDEVICE_ANALOGIN=1
)

add_subdirectory(../../mbed-os mbed-os)

add_library(sensor_filters STATIC
        debounce_to_digital_in.cpp
        smooth_to_analog_in.cpp
)

target_include_directories(sensor_filters PUBLIC include)

target_link_libraries(sensor_filters PUBLIC mbed-core-flags)