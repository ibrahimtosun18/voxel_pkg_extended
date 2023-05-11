cmake_minimum_required(VERSION 3.0.2)
project(voxel_pkg)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
)

find_package(PCL 1.7 REQUIRED COMPONENTS common io filters visualization)
find_package(PCL 1.7 REQUIRED COMPONENTS common io filters visualization surface)

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
# add_executable(voxel_up_node src/voxel_up_sampling/voxel_up_node.cpp src/voxel_up_sampling/Voxel_up.cpp)

add_executable(mls_upsampler_node src/mls_upsampler/mls_upsampler_node.cpp src/mls_upsampler/Mls_upsampler.cpp)

target_link_libraries(voxel_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(mls_upsampler_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


