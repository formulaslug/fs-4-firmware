cmake_minimum_required(VERSION 3.19.0)

set(APP_TARGET lib-mbed-ltc681x)

set(CMAKE_CXX_STANDARD 17)                                                                                                                   
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

file(GLOB_RECURSE _HDRS "include/*.h")

project(${APP_TARGET})

# Create library for bms
add_library(${APP_TARGET}
    ${_HDRS}
    src/LTC681xBus.cpp
    src/LTC681xChainBus.cpp
    src/LTC681xParallelBus.cpp
)
target_include_directories(${APP_TARGET} PUBLIC include)

target_link_libraries(${APP_TARGET} mbed-os)
