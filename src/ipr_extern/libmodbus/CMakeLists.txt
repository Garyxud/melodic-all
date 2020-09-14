cmake_minimum_required(VERSION 2.8.3)
project(libmodbus)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES modbus
)

###########
## Build ##
###########

include_directories(
  include
)

add_library(modbus
  src/modbus-data.c
  src/modbus-rtu.c
  src/modbus-tcp.c
  src/modbus.c
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS modbus
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
