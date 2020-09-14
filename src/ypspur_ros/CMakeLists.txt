cmake_minimum_required(VERSION 2.8.3)
project(ypspur_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation

  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
)

add_message_files(
  FILES
  DigitalOutput.msg
  DigitalInput.msg
  ControlMode.msg
  JointPositionControl.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp

  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
)
find_package(Boost REQUIRED COMPONENTS chrono thread atomic REQUIRED)

find_package(ypspur 1.17.0 REQUIRED)

###########
## Build ##
###########

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. ")
endif()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ypspur_INCLUDE_DIRS}
)

add_executable(ypspur_ros src/ypspur_ros.cpp)
target_link_libraries(ypspur_ros
  ${catkin_LIBRARIES}
  ypspur
)
add_dependencies(ypspur_ros ${ypspur_ros_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joint_tf_publisher src/joint_tf_publisher.cpp)
target_link_libraries(joint_tf_publisher ${catkin_LIBRARIES})
add_dependencies(joint_tf_publisher ${ypspur_ros_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(joint_position_to_joint_trajectory src/joint_position_to_joint_trajectory.cpp)
target_link_libraries(joint_position_to_joint_trajectory ${catkin_LIBRARIES})
add_dependencies(joint_position_to_joint_trajectory ${ypspur_ros_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS
    ypspur_ros
    joint_tf_publisher
    joint_position_to_joint_trajectory
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
