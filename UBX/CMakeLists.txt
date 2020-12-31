cmake_minimum_required(VERSION 3.10)

project(UBX)

add_library(UBX SHARED
    src/ubx_parser.cpp
    src/ubx_database.cpp
    src/ubx_message.cpp
    src/ubx_functions.cpp
    src/ubx_sfrbx_parser.cpp
    src/ubx_rxm_rawx.cpp
    src/ubx_listener.cpp)

target_include_directories(UBX PUBLIC include)
target_compile_options(UBX PUBLIC -std=c++17)

OPTION(BUILD_UBX_TESTS "Build UBX Tests" OFF)

IF(BUILD_UBX_TESTS)
    add_subdirectory(../gtest ../../gtest/build)
    
    add_executable(ubx_test
        test/test_ubx_database.cpp
        test/test_ubx_message.cpp
        test/test_ubx_parser.cpp
        test/test_ubx_functions.cpp
        test/test_ubx_rxm_rawx.cpp
        test/test_ubx_sfrbx_parser.cpp)
    target_link_libraries(ubx_test PRIVATE UBX)
    target_link_libraries(ubx_test PUBLIC gtest gtest_main)

ENDIF(BUILD_UBX_TESTS)