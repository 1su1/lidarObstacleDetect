cmake_minimum_required(VERSION 2.8.3)
project(lidar_obstacle_detection)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -lpthread")

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  message_filters
  jsk_rviz_plugins
  pcl_ros
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES lidar_detect
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/modules
  ${catkin_INCLUDE_DIRS}
)

# 添加子目录并构建
add_subdirectory(modules/pre_process/roi_clip)
add_subdirectory(modules/pre_process/voxel_grid_filter)
add_subdirectory(modules/euclidean_cluster)
add_subdirectory(modules/bounding_box)
# add_subdirectory(modules/ground_detector/patchwork)
add_subdirectory(modules/visualization)

add_executable(${PROJECT_NAME}_node main/lidar_obstacle_detection_node.cpp main/lidar_obstacle_detection.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  -lpthread
  roi_clip
  voxel_grid_filter
  euclidean_cluster
  bounding_box
  visualization
 )