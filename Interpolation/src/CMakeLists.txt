cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_read)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_read src/pcd_read.cpp)
target_link_libraries (pcd_read ${PCL_LIBRARIES})

add_executable (pcl_visualizer_demo src/pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})

add_executable (concatenate_clouds src/concatenate_clouds.cpp)
target_link_libraries (concatenate_clouds ${PCL_LIBRARIES}) 

add_executable (pcd_interpolate src/pcd_interpolate.cpp)
target_link_libraries (pcd_interpolate ${PCL_LIBRARIES})
