cmake_minimum_required(VERSION 3.10)
project(ublox)

if(NOT TARGET UBX)
    add_subdirectory(../UBX UBX)
endif(NOT TARGET UBX)

add_subdirectory(../cmake/async_comm async_comm)

add_library(UBLOX SHARED
    src/ublox.cpp)
target_include_directories(UBLOX PUBLIC include)
target_link_libraries(UBLOX PUBLIC async_comm UBX)