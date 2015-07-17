cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(dm2dpc)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (dm2dpc dm2dpc.cpp)
target_link_libraries (dm2dpc ${PCL_LIBRARIES})