cmake_minimum_required(VERSION 2.8.8)

project(qpoases_vendor)

find_package(catkin REQUIRED)

include(ExternalProject)
ExternalProject_Add(qpoases_3_2
  SVN_REPOSITORY https://projects.coin-or.org/svn/qpOASES/stable/3.2
  SVN_TRUST_CERT TRUE
  BUILD_IN_SOURCE TRUE
  INSTALL_COMMAND ""
)

ExternalProject_Get_property(qpoases_3_2 SOURCE_DIR)

set(qpOASES_CXX_LIBRARY ${SOURCE_DIR}/libs/libqpOASES.a)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qpOASES
)

install(DIRECTORY ${SOURCE_DIR}/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES ${qpOASES_CXX_LIBRARY}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
