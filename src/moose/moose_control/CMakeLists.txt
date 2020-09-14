cmake_minimum_required(VERSION 2.8.3)
project(moose_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  urdf
  realtime_tools
  tf
  nav_msgs
)

set(double_diff_drive_controller_sources
  src/double_diff_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES double_diff_drive_controller
  CATKIN_DEPENDS controller_interface realtime_tools
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(double_diff_drive_controller
  ${double_diff_drive_controller_headers} ${double_diff_drive_controller_sources})
target_link_libraries(double_diff_drive_controller ${catkin_LIBRARIES})

install(TARGETS double_diff_drive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES double_diff_drive_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/control.launch)
  roslaunch_add_file_check(launch/teleop.launch)
endif()
