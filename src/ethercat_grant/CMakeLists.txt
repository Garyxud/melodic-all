cmake_minimum_required(VERSION 2.8.3)
project(ethercat_grant)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ethercat_grant
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(ethercat_grant src/ethercat_grant.cpp)
target_link_libraries(ethercat_grant cap)

install(TARGETS ethercat_grant
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
