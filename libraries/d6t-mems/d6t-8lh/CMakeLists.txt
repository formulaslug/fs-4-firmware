cmake_minimum_required(VERSION 3.19)
project(d6t-8lh VERSION 1.0.0)

add_library(d6t-8lh STATIC
    d6t-8lh.cpp
    d6t-8lh.h
)

target_include_directories(d6t-8lh PUBLIC .)
target_link_libraries(d6t-8lh mbed-core-flags)
