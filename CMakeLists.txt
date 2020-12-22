cmake_minimum_required(VERSION 3.16)

project(ublox_read)

add_library(UBX 
    src/UBLOX/ubx_parser.cpp 
    src/UBLOX/ubx_database.cpp
    src/UBLOX/ubx_message.cpp)
target_include_directories(UBX PUBLIC include)

add_library(GNSS
    src/GNSS/nav_parser.cpp)
target_include_directories(GNSS PUBLIC include)

add_library(Bit_Utils
    src/Bit_Utils/bit_utils.cpp)
target_include_directories(Bit_Utils PUBLIC include)

include(ExternalProject)
ExternalProject_Add(async_comm
    PREFIX          lib/
    GIT_REPOSITORY  https://github.com/dpkoch/async_comm.git
    INSTALL_COMMAND cmake -E echo "Skipping install step"
    )

add_library(UBLOX
    src/UBLOX/configure.cpp)
target_include_directories(UBLOX PUBLIC include)

option(BUILD_TESTS "Build Tests" ON)
if(BUILD_TESTS)
    if(NOT TARGET gtest_main)
    # Download and unpack googletest at configure time
    configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
    if(result)
        message(FATAL_ERROR "CMake step for googletest failed: ${result}")
    endif(result)
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
    if(result)
        message(FATAL_ERROR "Build step for googletest failed: ${result}")
    endif(result)

    # Prevent overriding the parent project's compiler/linker
    # settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Add googletest directly to our build. This defines
    # the gtest and gtest_main targets.
    add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                        ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                        EXCLUDE_FROM_ALL)

    # The gtest/gtest_main targets carry header search path
    # dependencies automatically when using CMake 2.8.11 or
    # later. Otherwise we have to add them here ourselves.
    if (CMAKE_VERSION VERSION_LESS 2.8.11)
        include_directories("${gtest_SOURCE_DIR}/include")
    endif(CMAKE_VERSION VERSION_LESS 2.8.11)
    endif(NOT TARGET gtest_main)

    add_executable(ublox_test 
        test/test_ubx_parser.cpp 
        test/test_ubx_database.cpp
        test/test_ubx_message.cpp
        test/test_configure_ublox.cpp)
    target_link_libraries(ublox_test PUBLIC gtest gtest_main UBX UBLOX)
    
    add_executable(nav_test
        test/test_nav_parser.cpp)
    target_link_libraries(nav_test PUBLIC gtest gtest_main GNSS)

    add_executable(bit_tests
        test/test_bit_utils.cpp)
    target_link_libraries(bit_tests PUBLIC gtest gtest_main Bit_Utils)

endif(BUILD_TESTS)

option(BUILD_ROS "Build ROS Nodes, Messages, Services" ON)
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
