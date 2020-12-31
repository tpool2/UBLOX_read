cmake_minimum_required(VERSION 3.16)

project(ublox_read)

add_subdirectory(UBX)
add_subdirectory(GNSS)
add_subdirectory(Bit_Utils)
add_subdirectory(Configuration)

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
