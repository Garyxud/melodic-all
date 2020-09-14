cmake_minimum_required(VERSION 2.8.3)
project(pr2_gripper_sensor_controller)
find_package(catkin REQUIRED COMPONENTS pr2_controller_interface pr2_mechanism_model pr2_controllers_msgs realtime_tools pluginlib actionlib_msgs roscpp rosrt roslib std_msgs std_srvs pr2_gripper_sensor_msgs pr2_controller_manager)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})

catkin_package(
    DEPENDS pr2_controller_interface pr2_mechanism_model pr2_controllers_msgs realtime_tools pluginlib actionlib_msgs roscpp rosrt roslib std_msgs std_srvs pr2_gripper_sensor_msgs pr2_controller_manager
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include # TODO include
    LIBRARIES # TODO
)

add_library(pr2_gripper_sensor_controller_lib
  src/digitalFilter.cpp
  src/pr2_gripper_sensor_controller.cpp
  src/pressure_observer.cpp
  src/acceleration_observer.cpp)

target_link_libraries(pr2_gripper_sensor_controller_lib ${catkin_LIBRARIES})
add_dependencies(pr2_gripper_sensor_controller_lib ${catkin_EXPORTED_TARGETS} rosrt rosrt_EXPORTED_TARGETS)

install(TARGETS pr2_gripper_sensor_controller_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
