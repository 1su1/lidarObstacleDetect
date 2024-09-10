cmake_minimum_required(VERSION 2.8.3)
project(visualization)

find_package(autoware_auto_msgs REQUIRED)

aux_source_directory(. DIR_SRCS)

add_library(visualization ${DIR_SRCS})

