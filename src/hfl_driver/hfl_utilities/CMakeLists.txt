cmake_minimum_required(VERSION 3.5.1)

project(hfl_utilities)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

# Add hfl_utils library
add_library(${PROJECT_NAME} 
  src/base_hfl110dcu.cpp
  src/hfl_frame.cpp
  src/hfl_interface.cpp
  src/hfl_pixel.cpp
)

target_include_directories(${PROJECT_NAME}
  INTERFACE include
)

## Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
