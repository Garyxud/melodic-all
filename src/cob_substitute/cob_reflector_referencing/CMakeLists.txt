cmake_minimum_required(VERSION 2.8.3)
project(cob_reflector_referencing)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS bin/reflector_referencing_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
