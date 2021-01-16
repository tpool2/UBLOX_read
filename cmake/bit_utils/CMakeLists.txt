cmake_minimum_required(VERSION 3.10)

project(BIT_UTILS)

if(NOT TARGET bit_utils)
    configure_file(CMakeLists.txt.in bit_utils-download/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -g "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bit_utils-download )
    if(result)
        message(FATAL_ERROR "CMake step for bit_utils failed: ${result}")
    endif(result)
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bit_utils-download )
    if(result)
        message(FATAL_ERROR "Build step for bit_utils failed: ${result}")
    endif(result)

    add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/bit_utils-src
                        ${CMAKE_CURRENT_BINARY_DIR}/bit_utils-build
                        EXCLUDE_FROM_ALL)
                        
    
endif(NOT TARGET bit_utils)