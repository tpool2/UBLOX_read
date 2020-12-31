cmake_minimum_required(VERSION 3.10)

project(GNSS)

IF(NOT TARGET UBX)
        add_subdirectory(../UBX UBX)
ENDIF(NOT TARGET UBX)

add_library(GNSS SHARED
    src/nav_parser.cpp)
target_link_libraries(GNSS PUBLIC UBX)
target_compile_options(GNSS PRIVATE -std=c++17)
target_include_directories(GNSS PUBLIC include)

OPTION(BUILD_GNSS_TESTS "Build GNSS Tests" OFF)

IF(BUILD_GNSS_TESTS)
    add_subdirectory(../gtest ../../gtest/build)

    add_executable(gnss_test
        test/test_nav_parser.cpp)
    target_link_libraries(gnss_test PUBLIC gtest gtest_main)
    target_link_libraries(gnss_test PRIVATE GNSS)

ENDIF(BUILD_GNSS_TESTS)