cmake_minimum_required(VERSION 2.8.12)

# Don't set PROJECT_VERSION to empty string when no VERSION is given to project() command.
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 OLD)
endif()

project(rc_cloud_accumulator)
include(cmake/project_version.cmake)


# - Configuration for code optimization -

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build type: DEBUG or RELEASE" FORCE)
endif ()

include(cmake/optimization_flags.cmake)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2
  tf2_msgs
  tf2_ros
  std_srvs
)


find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)
add_definitions(${PCL_DEFINITIONS})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rc_slam_viewer
#  CATKIN_DEPENDS rc_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(rc_cloud_accumulator src/main.cc src/cloud_accumulator.cc src/cloud_visualizer.cc)
target_include_directories(rc_cloud_accumulator SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(rc_cloud_accumulator ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_options(rc_cloud_accumulator PRIVATE -Wall)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS rc_cloud_accumulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#  launch/start_demo.launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

######################################
## Define information for packaging ##
######################################
# optionally set description and maintainer (if not set will use defaults from package_debian.cmake)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Roboception viewer for the rc_visard")

# optionally set package maintainer (otherwise defaults to Roboception <info@roboception.de>)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Felix Endres <felix.endres@roboception.de>")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-$ENV{ROS_DISTRO}-rc-visard-driver, ros-$ENV{ROS_DISTRO}-pcl-ros")
include(cmake/package_debian.cmake)
