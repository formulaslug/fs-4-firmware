cmake_minimum_required(VERSION 4.1.1)
cmake_policy(VERSION 4.1.1)

set(MBED_APP_JSON_PATH mbed_app.json5)

include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)
project(BMStestbench
    LANGUAGES C CXX ASM)
include(mbed_project_setup)

add_subdirectory(mbed-os)
add_subdirectory(libraries/lib-mbed-ltc681x)


add_executable(${PROJECT_NAME} src/BmsConfig.h src/LTC6810.h src/Can.h src/BMSFaultDetection.h src/LTC6810.cpp src/main.cpp)
#add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} mbed-os lib-mbed-ltc681x) # Can also link to mbed-baremetal here
#target_link_libraries(${PROJECT_NAME} mbed-os)
mbed_set_post_build(${PROJECT_NAME})

