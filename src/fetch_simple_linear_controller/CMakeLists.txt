cmake_minimum_required(VERSION 2.8.3)
project(fetch_simple_linear_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(FILES
  LinearMove.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(linear_controller src/LinearController.cpp)
add_dependencies(linear_controller ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(linear_controller ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS linear_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
