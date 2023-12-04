cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(scene)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_to_gazebo src/pcl2gazebo.cpp)
target_link_libraries(pointcloud_to_gazebo ${PCL_LIBRARIES} tinyxml2)
