cmake_minimum_required(VERSION 3.0.2)
project(ur_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_package(
  CATKIN_DEPENDS rospy
)

## Mark executable scripts (Python etc.) for installationss
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  src/hand_tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)