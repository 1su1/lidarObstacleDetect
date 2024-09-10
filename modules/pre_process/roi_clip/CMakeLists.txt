cmake_minimum_required(VERSION 3.5)
project(roi_clip)

file(GLOB_RECURSE DIR_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

add_library(${PROJECT_NAME} STATIC ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME}
  ${pcl_ros_LIBRARIES}
)