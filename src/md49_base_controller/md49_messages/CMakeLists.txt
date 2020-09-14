cmake_minimum_required(VERSION 2.4.6)
project(md49_messages)

find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  message_generation
)

add_message_files(
  FILES 
  md49_encoders.msg
  md49_data.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  std_msgs 
  message_runtime
)
