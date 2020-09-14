cmake_minimum_required(VERSION 2.8.12)
project(optpp_catkin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

include(ExternalProject)

ExternalProject_Add(optpp
  GIT_REPOSITORY "https://github.com/ipab-slmc/optpp.git"
  GIT_TAG "master"
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=${CATKIN_DEVEL_PREFIX} --enable-static=no --enable-shared=yes --disable-warnings --includedir=${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME} 
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)
ExternalProject_Add_Step(optpp CopyToDevel
  COMMENT "Copying OPT++_config.h to install directory."
  COMMAND ${CMAKE_COMMAND} -E copy <BINARY_DIR>/include/OPT++_config.h ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}
  DEPENDEES install
)

if(IS_DIRECTORY ${CMAKE_INSTALL_PREFIX})
  message(WARNING "Compiling for install workspace")  
  catkin_package(
    INCLUDE_DIRS include/${PROJECT_NAME}
    LIBRARIES opt newmat
    CFG_EXTRAS optpp_extra.cmake
  )
else()
  message(WARNING "Compiling for devel workspace")
  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})
  catkin_package(
    INCLUDE_DIRS include ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES opt newmat
    CFG_EXTRAS optpp_extra.cmake
  )
endif()

install(
  DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
