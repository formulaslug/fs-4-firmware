cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19)

project(lap-counter LANGUAGES CXX)

add_library(lap-counter STATIC
    lap_counter.cpp
)

target_include_directories(lap-counter PUBLIC .)
