cmake_minimum_required(VERSION 2.8.3)
project(cartesian_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  CartesianCommand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    std_msgs
)
