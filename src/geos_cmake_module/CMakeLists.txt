cmake_minimum_required(VERSION 3.0.2)
project(geos_cmake_module)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS geos_cmake_module-extras.cmake)

## Install all cmake files
install(
	DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
