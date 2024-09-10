cmake_minimum_required(VERSION 2.8.3)
project(bounding_box)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(autoware_auto_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

aux_source_directory(. DIR_SRCS)

add_library(bounding_box STATIC ${DIR_SRCS})

target_link_libraries(bounding_box
  ${PCL_LIBRARIES}
 )

