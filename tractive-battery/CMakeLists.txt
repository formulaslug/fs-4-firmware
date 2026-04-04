cmake_minimum_required(VERSION 4.1.1)
cmake_policy(VERSION 4.1.1)

set(MBED_APP_JSON_PATH mbed_app.json5)
set(CUSTOM_TARGETS_JSON_PATH ../custom_targets/custom_targets.json5)
set(CUSTOM_UPLOAD_CFG_PATH ../custom_targets/UploadMethods.cmake)

include(../mbed-os/tools/cmake/mbed_toolchain_setup.cmake)
project(tractive-battery-board
    LANGUAGES C CXX ASM)
include(mbed_project_setup)


add_subdirectory(../custom_targets custom_targets)

add_subdirectory(../mbed-os mbed-os)
add_subdirectory(../libraries/lib-mbed-ltc681x lib-mbed-ltc681x)
add_subdirectory(../libraries/ds18b20 ds18b20)

add_executable(${PROJECT_NAME} BmsConfig.cpp BMSFaultDetection.cpp LTC6810.cpp main.cpp)
target_link_libraries(${PROJECT_NAME} mbed-os lib-mbed-ltc681x onewire-mbed-os ds18b20) # Can also link to mbed-baremetal here
mbed_set_post_build(${PROJECT_NAME})

