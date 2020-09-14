cmake_minimum_required(VERSION 2.8.3)
project(view_controller_msgs)

set(MSG_DEPS
  std_msgs
  geometry_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg message_generation ${MSG_DEPS})

set(MSG_FILES
CameraPlacement.msg
)

add_message_files(DIRECTORY msg FILES ${MSG_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS ${MSG_DEPS} message_runtime)

install(PROGRAMS
  scripts/CameraTest
  scripts/SquareTest
  scripts/ControlsTest
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
