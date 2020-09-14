# Copyright (c) 2019, The MITRE Corporation
# All rights reserved.
#
# Sections of this project contains content developed by The MITRE Corporation.
# If this code is used in a deployment or embedded within another project,
# it is requested that you send an email to opensource@mitre.org in order to
# let us know where this software is being used.

cmake_minimum_required(VERSION 2.8.3)
project(kvh_geo_fog_3d_rviz)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  kvh_geo_fog_3d_msgs
  roslint
  rviz
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# QT
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if("${rviz_QT_VERSION}" VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED COMPONENTS QtCore QtGui)
  include(${QT_USE_FILE})
  qt4_wrap_cpp(QT_MOC include/kvh_geo_fog_3d_status_panel.hpp include/kvh_status_painter.hpp)
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED COMPONENTS Core Widgets)
  set(QT_LIBRARIES Qt5::Core Qt5::Widgets)
  qt5_wrap_cpp(QT_MOC include/kvh_geo_fog_3d_status_panel.hpp include/kvh_status_painter.hpp)
endif()

add_definitions(-DQT_NO_KEYWORDS)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS diagnostic_msgs kvh_geo_fog_3d_msgs rviz
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/kvh_geo_fog_3d_status_panel.cpp
  src/kvh_status_painter.cpp
  ${QT_MOC}
)

# Add cmake target dependencies of the library
# as an example, code may need to be generated before libraries
# either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add roslint target
# Note that roslint requires explicit lists of source/header
# files, which is not ideal. There's an issue open from 2014
# about this (https://github.com/ros/roslint/issues/27)
#
# Ignore ROS's suggestions for:
# * Comment spacing
# * Parentesis whitespace
set(ROSLINT_CPP_OPTS "--filter=-whitespace/comments,-whitespace/parens")
roslint_cpp(
  src/kvh_geo_fog_3d_status_panel.cpp
  src/kvh_status_painter.cpp
  include/kvh_geo_fog_3d_status_panel.hpp
  include/kvh_status_painter.hpp
)

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ublox_rviz.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
