cmake_minimum_required(VERSION 3.19)
project(d6t-1a VERSION 1.0.0)

add_library(d6t-1a STATIC
    d6t-1a.cpp
    d6t-1a.h
)

target_include_directories(d6t-1a PUBLIC .)
target_link_libraries(d6t-1a mbed-core-flags)
