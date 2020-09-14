cmake_minimum_required(VERSION 2.8.3)
project(multimaster_launch)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch)
  roslaunch_add_file_check(launch)
endif()
