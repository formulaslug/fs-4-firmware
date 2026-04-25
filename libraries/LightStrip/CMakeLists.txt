cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19)

set(CMAKE_CXX_STANDARD 20)

project(light_strip LANGUAGES CXX)

add_library(light_strip STATIC
        light_strip.cpp
        light_strip_exec.cpp
)

target_include_directories(light_strip PUBLIC include)

target_link_libraries(light_strip PUBLIC mbed-core-flags)
