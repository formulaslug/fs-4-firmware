cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19)

set(CMAKE_CXX_STANDARD 17)

project(filter-utils LANGUAGES CXX)

# add_definitions(
#         -DDEVICE_ANALOGIN=1
# )

# add_subdirectory(../../mbed-os mbed-os)

add_library(filter-utils STATIC
        debounced_digital_in.cpp
        filtered_analog_in.cpp
        low_pass_filter.cpp
)

target_include_directories(filter-utils PUBLIC include)

target_link_libraries(filter-utils PUBLIC mbed-core-flags)
