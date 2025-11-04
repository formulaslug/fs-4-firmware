add_library(util STATIC
    src/util.cpp
)

target_include_directories(util_library PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(util_library PRIVATE cxx_std_17)