cmake_minimum_required(VERSION 2.8.3)
project(robot_activity_tutorials)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  std_srvs
  robot_activity
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_activity_tutorials
  CATKIN_DEPENDS roscpp std_srvs robot_activity
)

set(ROSLINT_CPP_OPTS "--filter=+,-build/c++11,-runtime/references,-build/storage_class")
roslint_cpp()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(robot_activity_tutorials src/robot_activity_tutorials.cpp)
add_dependencies(robot_activity_tutorials ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_activity_tutorials ${catkin_LIBRARIES})

add_executable(robot_activity_tutorials_node src/robot_activity_tutorials_node.cpp)
add_dependencies(robot_activity_tutorials_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_activity_tutorials_node robot_activity_tutorials)

add_executable(robot_activity_tutorials_minimal_node src/robot_activity_tutorials_minimal_node.cpp)
add_dependencies(robot_activity_tutorials_minimal_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_activity_tutorials_minimal_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS robot_activity_tutorials
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(TARGETS
  robot_activity_tutorials_node
  robot_activity_tutorials_minimal_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
