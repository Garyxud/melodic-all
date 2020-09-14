cmake_minimum_required(VERSION 2.8.3)
project(pr2eus_tutorials)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY euslisp launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )


