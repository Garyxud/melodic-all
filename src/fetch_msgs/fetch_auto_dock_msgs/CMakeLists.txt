cmake_minimum_required(VERSION 2.8.3)
project(fetch_auto_dock_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    actionlib_msgs
    geometry_msgs
    message_generation
)

add_action_files(
  FILES
    Dock.action
    Undock.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    message_runtime
)
