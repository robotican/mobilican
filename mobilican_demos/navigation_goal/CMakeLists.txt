cmake_minimum_required(VERSION 2.8.3)
project(navigation_goals)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  genmsg 
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation_goals
#  CATKIN_DEPENDS actionlib move_base_msgs roscpp
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_base_client src/move_base_client.cpp)
target_link_libraries(move_base_client ${catkin_LIBRARIES})
add_dependencies(move_base_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})