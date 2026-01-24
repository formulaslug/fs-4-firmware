cmake_minimum_required(VERSION 3.19)
project(ds18b20 VERSION 1.0.0)

add_subdirectory(../onewire-mbed-os onewire-mbed-os)

add_library(ds18b20 STATIC DS18B20.cpp)
target_include_directories(ds18b20 PUBLIC .)
target_link_libraries(ds18b20 mbed-core-flags onewire-mbed-os)
