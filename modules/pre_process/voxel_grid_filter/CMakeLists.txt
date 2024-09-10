cmake_minimum_required(VERSION 2.8.3)
project(voxel_grid_filter)

aux_source_directory(. DIR_SRCS)

add_library(voxel_grid_filter STATIC ${DIR_SRCS})

target_link_libraries(voxel_grid_filter
  ${PCL_LIBRARIES}
)
