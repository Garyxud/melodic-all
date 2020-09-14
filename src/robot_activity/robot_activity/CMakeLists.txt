cmake_minimum_required(VERSION 2.8.3)
project(robot_activity)

add_compile_options(-std=c++11)
add_compile_options(-Wall)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  std_srvs
  robot_activity_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_activity
  CATKIN_DEPENDS roscpp std_srvs robot_activity_msgs
)

###########
## Build ##
###########

set(ROSLINT_CPP_OPTS "--filter=+,-build/c++11,-runtime/references,-build/storage_class")
roslint_cpp()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(robot_activity
  src/robot_activity.cpp
  src/managed_robot_activity.cpp
  src/resource/managed_resource.cpp
  src/resource/resource_manager.cpp
)
add_dependencies(robot_activity ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_activity ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS robot_activity
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_rostest_gtest(
    robot_activity_tests
    test/robot_activity.test
    test/robot_activity_tests.cpp)
  add_dependencies(robot_activity_tests ${catkin_EXPORTED_TARGETS})
  target_link_libraries(robot_activity_tests robot_activity ${catkin_LIBRARIES})

  add_executable(any_robot_activity test/any_robot_activity.cpp)
  add_dependencies(any_robot_activity ${catkin_EXPORTED_TARGETS})
  target_link_libraries(any_robot_activity robot_activity ${catkin_LIBRARIES})
  add_rostest(test/heartbeat.test)

  roslint_add_test()

endif()
