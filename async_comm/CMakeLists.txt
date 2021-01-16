cmake_minimum_required(VERSION 3.10)

project(ASYNC_COMM)

if(NOT TARGET async_comm)
    configure_file(CMakeLists.txt.in async_comm-download/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -g "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/async_comm-download )
    if(result)
        message(FATAL_ERROR "CMake step for async_comm failed: ${result}")
    endif(result)
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/async_comm-download )
    if(result)
        message(FATAL_ERROR "Build step for async_comm failed: ${result}")
    endif(result)

    add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/async_comm-src
                        ${CMAKE_CURRENT_BINARY_DIR}/async_comm-build
                        EXCLUDE_FROM_ALL)
                        
    
endif(NOT TARGET async_comm)