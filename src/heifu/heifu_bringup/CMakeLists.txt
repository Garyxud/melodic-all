cmake_minimum_required(VERSION 2.8.3)
project(heifu_bringup)
find_package(catkin REQUIRED COMPONENTS)

catkin_package()
# install

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
