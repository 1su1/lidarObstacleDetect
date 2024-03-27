cmake_minimum_required(VERSION 3.0)
project(patchworkpp)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
)


add_library(${PROJECT_NAME}  STATIC  patchworkpp.cpp)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)