cmake_minimum_required(VERSION 2.8.3)

project(marti_dbw_msgs)

set(MSG_DEPS 
  std_msgs)

set(BUILD_DEPS
  ${MSG_DEPS} 
  message_generation)

set(RUNTIME_DEPS
  ${MSG_DEPS} 
  message_runtime)

### Catkin ###
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})

### Message Generation ###
add_message_files(DIRECTORY msg FILES
  PrimaryControl.msg
  PrimaryFeedback.msg
  TransmissionFeedback.msg)
generate_messages(DEPENDENCIES ${MSG_DEPS})

### More Catkin ###
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${RUNTIME_DEPS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
