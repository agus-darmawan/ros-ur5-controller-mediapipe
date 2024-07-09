cmake_minimum_required(VERSION 3.0.2)
project(ur_srvs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StartService.srv
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)