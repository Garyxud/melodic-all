cmake_minimum_required(VERSION 2.8.3)
project(dynamic_robot_state_publisher)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS dynamic_reconfigure kdl_parser robot_state_publisher roscpp sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/DynamicRobotState.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure kdl_parser robot_state_publisher roscpp sensor_msgs
)

# robot_state_publisher doesn't export the joint_state_listener library via
# catkin_package LIBRARIES, so we have to find and add it ourselves
find_library(JOINT_STATE_LISTENER_LIB joint_state_listener HINTS ${catkin_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/dynamic_robot_state_publisher_node.cpp src/joint_state_listener.cpp src/robot_state_publisher.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${JOINT_STATE_LISTENER_LIB})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)