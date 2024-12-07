cmake_minimum_required(VERSION 3.0.2)
project(my_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  visualization_msgs
  tf
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable for stanley_controller
add_executable(stanley_controller src/stanley_controller.cpp)
target_link_libraries(stanley_controller ${catkin_LIBRARIES})

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/mpccontrol.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
