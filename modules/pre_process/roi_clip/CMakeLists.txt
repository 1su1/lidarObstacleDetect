cmake_minimum_required(VERSION 2.8.3)
project(roi_clip)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

#SHARED 动态库
#STATIC 静态库
add_library(roi_clip STATIC ${DIR_SRCS})

target_link_libraries(roi_clip
  ${PCL_LIBRARIES}
)

 