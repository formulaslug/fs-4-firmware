cmake_minimum_required(VERSION 3.19)
project(onewire-mbed-os VERSION 1.0.0)

add_library(onewire-mbed-os STATIC
    OneWire.cpp
)
target_include_directories(onewire-mbed-os PUBLIC .)
target_link_libraries(onewire-mbed-os mbed-core-flags)
