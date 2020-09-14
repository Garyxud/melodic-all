cmake_minimum_required(VERSION 2.8.3)
project(tf2_relay)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_relay
  tf2_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES tf2_relay
 CATKIN_DEPENDS tf2_msgs message_relay
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(tf2_relay
  src/transform_relay.cpp
)
target_link_libraries(tf2_relay
  ${catkin_LIBRARIES}
)

add_executable(tf2_relay_node src/tf2_relay_node.cpp)
target_link_libraries(tf2_relay_node
  tf2_relay ${catkin_LIBRARIES}
)

install(TARGETS tf2_relay tf2_relay_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  roslaunch_add_file_check(launch)
  roslint_cpp()
  roslint_add_test()
endif()
