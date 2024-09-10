cmake_minimum_required(VERSION 3.5)
project(lidar_obstacle_detection)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -lpthread")

# Set build type to Release
set(CMAKE_BUILD_TYPE Release)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/modules
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add subdirectories for additional modules
add_subdirectory(modules/pre_process/roi_clip)
add_subdirectory(modules/pre_process/voxel_grid_filter)
# add_subdirectory(modules/euclidean_cluster)
# add_subdirectory(modules/bounding_box)
# add_subdirectory(modules/visualization)

# Add executable and link libraries
add_executable(${PROJECT_NAME}_node main/main.cpp main/lidar_obstacle_detection.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  roi_clip
  voxel_grid_filter
  # euclidean_cluster
  # bounding_box
  # visualization
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  -lpthread
)

# Install
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()