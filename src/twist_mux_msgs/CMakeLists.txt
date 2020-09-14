cmake_minimum_required(VERSION 2.8.3)
project(twist_mux_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib actionlib_msgs)

include_directories(include)

add_action_files(
  DIRECTORY action
  FILES
    JoyPriority.action
    JoyTurbo.action)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
