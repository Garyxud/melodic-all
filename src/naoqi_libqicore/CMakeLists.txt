## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.

cmake_minimum_required(VERSION 2.8)
project(naoqi_libqicore)
find_package(catkin REQUIRED naoqi_libqi)
find_package(Boost)

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})

catkin_package(LIBRARIES qicore
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/libqicore/
  DEPENDS Boost
)

# Tests
enable_testing()

add_subdirectory("libqicore")

# Install catkin package.xml
install(FILES package.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})