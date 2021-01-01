cmake_minimum_required(VERSION 3.10)

project(Bit_Utils)

add_library(Bit_Utils SHARED
    src/bit_utils.cpp)
target_compile_options(Bit_Utils PUBLIC -std=c++17)
target_include_directories(Bit_Utils PUBLIC include)

OPTION(BUILD_BIT_UTILS_TESTS "Build Bit Utils Tests" ON)

IF(BUILD_BIT_UTILS_TESTS)
    if(NOT TARGET gtest)
        add_subdirectory(../gtest ../../gtest/build)
    endif(NOT TARGET gtest)

    add_executable(bit_tests
        test/test_bit_utils.cpp)
        target_link_libraries(bit_tests PUBLIC gtest gtest_main Bit_Utils)

ENDIF(BUILD_BIT_UTILS_TESTS)