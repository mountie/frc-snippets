cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PCL_DIR "/opt/pointclouds/share/pcl-1.8")

project(pcd_write)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

add_executable (grabber grabber.cpp)
target_link_libraries (grabber ${PCL_LIBRARIES})

add_executable (benchmark benchmark.cpp)
target_link_libraries (benchmark ${PCL_LIBRARIES})

add_executable (viewer viewer.cpp)
target_link_libraries (viewer ${PCL_LIBRARIES})
target_link_libraries (viewer -lfreenect)
target_link_libraries (viewer -lfreenect_sync)

add_executable (openni openni.cpp)
target_link_libraries (openni ${PCL_LIBRARIES})

add_executable (showpcd showpcd.cpp)
target_link_libraries (showpcd ${PCL_LIBRARIES})
