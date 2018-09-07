cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(registration_NDT)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (registration_NDT registration_NDT.cpp)
target_link_libraries (registration_NDT ${PCL_LIBRARIES})