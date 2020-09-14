cmake_minimum_required(VERSION 2.8.3)
project(mikrotik_swos_tools)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

install(PROGRAMS
  nodes/swos_api
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)