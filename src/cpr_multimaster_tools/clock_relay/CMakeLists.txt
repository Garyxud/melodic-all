cmake_minimum_required(VERSION 2.8.3)
project(clock_relay)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_relay
  rosgraph_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES clock_relay
 CATKIN_DEPENDS rosgraph_msgs message_relay
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(clock_relay
  src/clock_relay.cpp
)
target_link_libraries(clock_relay
  ${catkin_LIBRARIES}
)

add_executable(clock_relay_node src/clock_relay_node.cpp)
target_link_libraries(clock_relay_node
  clock_relay ${catkin_LIBRARIES}
)

install(TARGETS clock_relay clock_relay_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()
