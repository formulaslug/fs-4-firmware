cmake_minimum_required(VERSION 3.19)
project(bt817q-dash VERSION 1.0.0)

add_library(bt817q STATIC
    BT817Q.cpp
)
target_include_directories(bt817q PUBLIC .)
target_link_libraries(bt817q mbed-core-flags)
