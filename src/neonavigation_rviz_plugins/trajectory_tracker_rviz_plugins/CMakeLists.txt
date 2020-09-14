cmake_minimum_required(VERSION 3.0.2)
project(trajectory_tracker_rviz_plugins)

set(CATKIN_DEPENDS
  pluginlib
  rviz
  trajectory_tracker_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
catkin_package(CATKIN_DEPENDS ${CATKIN_DEPENDS})


add_compile_options(-std=c++11)
include_directories(include ${catkin_INCLUDE_DIRS})
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)


if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})

  qt4_wrap_cpp(MOC_FILES
    include/trajectory_tracker_rviz_plugins/path_with_velocity_display.h
  )
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)

  qt5_wrap_cpp(MOC_FILES
    include/trajectory_tracker_rviz_plugins/path_with_velocity_display.h
  )
endif()

find_file(HAVE_VALIDATE_QUATERNION_H
  NAMES rviz/validate_quaternions.h
  HINTS ${catkin_INCLUDE_DIRS}
)
if(HAVE_VALIDATE_QUATERNION_H)
  add_definitions(-DHAVE_VALIDATE_QUATERNION_H)
endif()


add_library(${PROJECT_NAME}
  src/path_with_velocity_display.cpp
  ${MOC_FILES}
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
