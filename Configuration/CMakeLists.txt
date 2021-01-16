cmake_minimum_required(VERSION 3.10)

project(Configuration)

if(NOT TARGET UBX)
    add_subdirectory(../UBX UBX)
endif(NOT TARGET UBX)

add_subdirectory(../cmake/async_comm async_comm)

add_library(CONFIGURATION SHARED
    src/configure.cpp)
target_include_directories(CONFIGURATION PUBLIC include)
target_link_libraries(CONFIGURATION PUBLIC UBX async_comm)

add_executable(cfg_val_set src/cfg_val_set.cpp)
target_link_libraries(cfg_val_set PUBLIC CONFIGURATION)

add_executable(cfg_val_get src/cfg_val_get.cpp)
target_link_libraries(cfg_val_get PUBLIC CONFIGURATION)

OPTION(BUILD_CONFIGURATION_TESTS "Build Configuration Tests" OFF)
IF(BUILD_CONFIGURATION_TESTS)
    add_subdirectory(../cmake/gtest ../../gtest/build)

    add_executable(configuration_tests
        test/test_configure_ublox.cpp)
    target_link_libraries(configuration_tests PUBLIC gtest gtest_main CONFIGURATION)

ENDIF(BUILD_CONFIGURATION_TESTS)