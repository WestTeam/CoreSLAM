# Copyright (c) 2018 All Rights Reserved WestBot

cmake_minimum_required( VERSION 2.8.1 )

find_package( OpenCV REQUIRED )

project( CoreSlam )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11" )
set( CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g" )

SET( CMAKE_C_COMPILER "/usr/bin/g++" )

include_directories( include )

# library
add_library(
    CoreSLAM SHARED
    src/CoreSlamLoopClosing.cpp
    src/CoreSlamState.cpp
    src/Map.cpp
    src/Position.cpp
    src/Randomizer.cpp )

target_link_libraries( CoreSLAM )

# Binaries
add_executable( TestLabReverse test/TestLabReverse.cpp )

target_link_libraries(
    TestLabReverse
	${OpenCV_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/build/libCoreSLAM.dylib )

add_custom_command(
    TARGET TestLabReverse POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_SOURCE_DIR}/data/test_lab.dat
    ${PROJECT_SOURCE_DIR}/build/test_lab.dat )

add_dependencies( TestLabReverse CoreSLAM )
