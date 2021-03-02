cmake_minimum_required(VERSION 3.10)

project(GNSS)

if(NOT TARGET UBX)
    add_subdirectory(../UBX UBX)
endif(NOT TARGET UBX)

add_subdirectory(../cmake/bit_utils bit_utils)

add_library(GNSS SHARED
    src/nav_parser.cpp
    src/ephemeris.cpp
    src/gps_lnav.cpp
    src/gps_cnav.cpp
    src/gps_cei.cpp
    src/satellite.cpp)
target_link_libraries(GNSS PUBLIC UBX bit_utils)
target_compile_options(GNSS PRIVATE -std=c++17)
target_include_directories(GNSS PUBLIC include)

OPTION(BUILD_GNSS_TESTS "Build GNSS Tests" OFF)

IF(BUILD_GNSS_TESTS)
    add_subdirectory(../cmake/gtest ../../gtest/build)

    add_executable(gnss_test
        test/test_nav_parser.cpp
        test/test_gps_lnav.cpp
        test/test_gps_cnav.cpp
        test/test_gps_cei.cpp)
    target_link_libraries(gnss_test PUBLIC gtest gtest_main)
    target_link_libraries(gnss_test PRIVATE GNSS)

ENDIF(BUILD_GNSS_TESTS)