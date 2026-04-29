cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19)

set(CMAKE_CXX_STANDARD 17)

project(bno086 LANGUAGES CXX)

add_definitions(
        -DDEVICE_ANALOGIN=1
)

# add_subdirectory(../../mbed-os mbed-os)

add_library(bno086 STATIC
        BNO080.cpp
        BNO080Async.cpp
)

target_include_directories(bno086 PUBLIC include)

target_link_libraries(bno086 PUBLIC mbed-core-flags)