# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesPointcloud)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-pointcloud rs-pointcloud.cpp ../example.hpp " register.h")
    #add_executable(realtime_track realtime_track.cpp ../example.hpp " register.h")
    #add_executable(rs-pcl rs-pcl.cpp ../example.hpp)
    set_property(TARGET rs-pointcloud PROPERTY CXX_STANDARD 11)
    target_link_libraries(rs-pointcloud ${DEPENDENCIES} ${PCL_LIBRARIES})
    include_directories(../)
    set_target_properties (rs-pointcloud PROPERTIES FOLDER Examples)
    install(TARGETS rs-pointcloud RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(realtime_track realtime_track.cpp ../example.hpp " register.h")
    #add_executable(realtime_track realtime_track.cpp ../example.hpp " register.h")
    #add_executable(rs-pcl rs-pcl.cpp ../example.hpp)
    set_property(TARGET realtime_track PROPERTY CXX_STANDARD 11)
    target_link_libraries(realtime_track ${DEPENDENCIES} ${PCL_LIBRARIES})
    include_directories(../)
    set_target_properties (realtime_track PROPERTIES FOLDER Examples)
    install(TARGETS realtime_track RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()


