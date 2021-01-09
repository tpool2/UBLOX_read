cmake_minimum_required(VERSION 3.10)

project(GNSS)

if(NOT TARGET UBX)
    add_subdirectory(../UBX UBX)
endif(NOT TARGET UBX)

if(NOT TARGET Bit_Utils)
    add_subdirectory(../Bit_Utils Bit_Utils)
endif(NOT TARGET Bit_Utils)

add_library(GNSS SHARED
    src/nav_parser.cpp
    src/satellite.cpp
    src/ephemeris.cpp)
target_link_libraries(GNSS PUBLIC UBX Bit_Utils)
target_compile_options(GNSS PRIVATE -std=c++17)
target_include_directories(GNSS PUBLIC include)

OPTION(BUILD_GNSS_TESTS "Build GNSS Tests" ON)

IF(BUILD_GNSS_TESTS)
    add_subdirectory(../gtest ../../gtest/build)

    add_executable(gnss_test
        test/test_nav_parser.cpp
        test/test_satellite.cpp
        test/test_ephemeris.cpp)
    target_link_libraries(gnss_test PUBLIC gtest gtest_main)
    target_link_libraries(gnss_test PRIVATE GNSS)

ENDIF(BUILD_GNSS_TESTS)