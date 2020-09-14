cmake_minimum_required(VERSION 2.8.3)
project(seed_smartactuator_sdk)

find_package(catkin REQUIRED COMPONENTS)

# add c++ dependencies :
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c+11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCXX11_SUPPORTED)
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCXX11_SUPPORTED)
else()
  message(FATAL "c++11 required but not supported")
endif()

# add boost dependencies :
find_package(Boost REQUIRED COMPONENTS filesystem system signals thread)
if(NOT Boost_INCLUDE_DIRS)
  message(FATAL "boost required but not supported")
else()
  include_directories(include ${Boost_INCLUDE_DIRS})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES seed_smartactuator_sdk
)

catkin_python_setup()

include_directories(  ${catkin_INCLUDE_DIRS})

add_library(seed_smartactuator_sdk
  src/seed3_command.cpp
  src/aero3_command.cpp
  )
target_link_libraries(seed_smartactuator_sdk ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(seed_smartactuator_sdk ${catkin_EXPORTED_TARGETS})

install(TARGETS seed_smartactuator_sdk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

