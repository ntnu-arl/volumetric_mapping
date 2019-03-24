cmake_minimum_required(VERSION 2.8.3)
project(octomap_world)

find_package(catkin_simple REQUIRED)
find_package(dynamicEDT3D)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

include_directories(
  ${DYNAMICEDT3D_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/octomap_world.cc
  src/octomap_manager.cc
)

############
# BINARIES #
############
cs_add_executable(octomap_manager
  src/octomap_manager_node.cc
)
target_link_libraries(octomap_manager ${PROJECT_NAME} ${DYNAMICEDT3D_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${DYNAMICEDT3D_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
