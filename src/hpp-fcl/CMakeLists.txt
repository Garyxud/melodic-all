cmake_minimum_required(VERSION 2.8.12)
project(hpp-fcl)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(octomap REQUIRED)
find_package(assimp REQUIRED)

include(ExternalProject)

ExternalProject_Add(hpp-fcl_src
  GIT_REPOSITORY "https://github.com/humanoid-path-planner/hpp-fcl.git"
  GIT_TAG "v1.0.1"

  CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INSTALL_PREFIX:PATH=${PROJECT_BINARY_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib
)
ExternalProject_Add_Step(hpp-fcl_src CopyToDevel
  COMMENT "Copying to devel"
  COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_BINARY_DIR}/install/lib/libhpp-fcl.so ${CATKIN_DEVEL_PREFIX}/lib/
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_BINARY_DIR}/install/include/hpp/ ${CATKIN_DEVEL_PREFIX}/include/hpp/
  DEPENDEES install
)

if(IS_DIRECTORY ${CMAKE_INSTALL_PREFIX})
  message(STATUS "Compiling for install workspace (${CMAKE_INSTALL_PREFIX})")
  catkin_package(
    INCLUDE_DIRS include/${PROJECT_NAME}/
    LIBRARIES hpp-fcl ${ASSIMP_LIBRARIES}
    # DEPENDS Boost OCTOMAP ASSIMP
    CFG_EXTRAS hpp-fcl.cmake
  )
else()
  message(STATUS "Compiling for devel workspace")
  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/hpp/fcl)
  catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/
    LIBRARIES hpp-fcl ${ASSIMP_LIBRARIES}
    # DEPENDS Boost OCTOMAP ASSIMP
    CFG_EXTRAS hpp-fcl.cmake
  )
endif()

install(
  DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  FILES ${CATKIN_DEVEL_PREFIX}/lib/libhpp-fcl.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
