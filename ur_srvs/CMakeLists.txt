cmake_minimum_required(VERSION 3.0.2)
project(ur_srvs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StartService.srv
  ModeService.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)