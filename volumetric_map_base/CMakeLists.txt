cmake_minimum_required(VERSION 3.0.2)
project(volumetric_map_base)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/world_base.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
