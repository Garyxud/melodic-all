cmake_minimum_required(VERSION 2.8.3)
project(heifu_tools)

find_package(catkin REQUIRED COMPONENTS
  
)

# catkin_python_setup()

catkin_package(
  #  INCLUDE_DIRS include
)

# TODO: Install Rules
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)


