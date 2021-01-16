cmake_minimum_required(VERSION 3.10)

project(Configuration)

if(NOT TARGET UBX)
    add_subdirectory(../UBX UBX)
endif(NOT TARGET UBX)

if(NOT TARGET async_comm)
    add_subdirectory(../async_comm async_comm)
endif(NOT TARGET async_comm)

add_library(CONFIGURATION SHARED
    src/configure.cpp)
target_include_directories(CONFIGURATION PUBLIC include)
target_link_libraries(CONFIGURATION PUBLIC UBX async_comm)

OPTION(BUILD_CONFIGURATION_TESTS "Build Configuration Tests" OFF)

IF(BUILD_CONFIGURATION_TESTS)
    add_subdirectory(../gtest ../../gtest/build)

    add_executable(configuration_tests
        test/test_configure_ublox.cpp)
    target_link_libraries(configuration_tests PUBLIC gtest gtest_main CONFIGURATION)

ENDIF(BUILD_CONFIGURATION_TESTS)