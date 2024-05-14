cmake_minimum_required(VERSION 3.0.2)
project(voxel_pkg)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
)

find_package(PCL 1.7 REQUIRED COMPONENTS common io filters visualization)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES voxel_pkg
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(voxel_filter_node src/voxel_down_sampling/voxel_filter_node.cpp src/voxel_down_sampling/Voxel_filter.cpp)

target_link_libraries(voxel_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} stdc++fs)
