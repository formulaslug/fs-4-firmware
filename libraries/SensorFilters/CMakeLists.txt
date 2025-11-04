cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19)

set(CMAKE_CXX_STANDARD 17)

project(sensor-filters LANGUAGES CXX)

add_definitions(
        -DDEVICE_ANALOGIN=1
)

add_subdirectory(../../mbed-os mbed-os)

add_library(sensor-filters STATIC
        debounce_to_digital_in.cpp
        smooth_to_analog_in.cpp
)

target_include_directories(sensor-filters PUBLIC include)

target_link_libraries(sensor-filters PUBLIC mbed-core-flags)