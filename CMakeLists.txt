cmake_minimum_required(VERSION 3.16)

project(ublox_read)

add_subdirectory(UBX)
add_subdirectory(GNSS)
add_subdirectory(Configuration)

add_executable(read_file
    src/executables/read_file.cpp)
target_link_libraries(read_file PUBLIC UBX GNSS)

add_executable(orb_read
    src/executables/orb_read.cpp)
target_link_libraries(orb_read PUBLIC UBX GNSS)

option(BUILD_ROS "Build ROS Nodes, Messages, Services" OFF)
if (${BUILD_ROS})
    message(STATUS "BUILDING UBLOX ROS PACKAGE")
    find_package(
        catkin REQUIRED COMPONENTS roscpp message_generation
    )
    catkin_package(
        LIBRARIES UBX
        INCLUDE_DIRS include
        CATKIN_DEPENDS std_msgs message_runtime
    )
endif(${BUILD_ROS})
