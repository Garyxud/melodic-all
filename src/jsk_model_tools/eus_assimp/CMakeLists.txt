# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(eus_assimp)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS euslisp)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")

find_package(PkgConfig)
pkg_check_modules(assimpdevel assimp_devel REQUIRED)
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
if(NOT euslisp_INCLUDE_DIRS)
  if(EXISTS ${euslisp_SOURCE_DIR}/jskeus)
    set(euslisp_PACKAGE_PATH ${euslisp_SOURCE_DIR})
  else()
    set(euslisp_PACKAGE_PATH ${euslisp_PREFIX}/share/euslisp)
  endif()
  message("-- Set euslisp_PACKAGE_PATH to ${euslisp_PACKAGE_PATH}")
  set(euslisp_INCLUDE_DIRS ${euslisp_PACKAGE_PATH}/jskeus/eus/include)
endif()
message("-- Set euslisp_INCLUDE_DIRS to ${euslisp_INCLUDE_DIRS}")
include_directories(/usr/include /usr/X11R6/include ${euslisp_INCLUDE_DIRS} ${assimpdevel_INCLUDE_DIRS})
link_directories(${assimpdevel_LIBRARY_DIRS})
add_library(eus_assimp src/eus_assimp.cpp)

target_link_libraries(eus_assimp ${assimpdevel_LIBRARIES})
set_target_properties(eus_assimp PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)
add_dependencies(eus_assimp libassimp_devel)

# compile flags
add_definitions(-O2 -Wno-write-strings -Wno-comment)
add_definitions(-Di486 -DLinux -D_REENTRANT -DVERSION='\"9.00\"' -DTHREADED -DPTHREAD -DX11R6_1)
add_definitions('-DREPOVERSION="\\"${REPOVERSION}\\""')
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES aarch64* )
 add_definitions(-Dx86_64)
else()
 add_definitions(-Di486)
endif()

## for image file loading using SDL
pkg_check_modules (SDLIMG SDL_image)
if(SDLIMG_FOUND)
#  message(STATUS "SDL_image found")
  add_definitions(-DUSE_SDL_IMAGE=1)
  add_definitions(${SDLIMG_CFLAGS_OTHER})
  include_directories(${SDLIMG_INCLUDE_DIRS})
  target_link_libraries(eus_assimp ${SDLIMG_LIBRARIES})
endif(SDLIMG_FOUND)

if(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
 add_definitions(-Dx86_64)
 set(CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS} -flat_namespace -undefined suppress")
endif()

set_target_properties(eus_assimp PROPERTIES PREFIX "" SUFFIX ".so")

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS
    CATKIN_DEPENDS # euslisp TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES assimp_devel # TODO
)

# install
install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
