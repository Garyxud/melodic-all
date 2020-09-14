cmake_minimum_required(VERSION 2.8.3)
project(dataspeed_ulc_can)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  std_msgs
  geometry_msgs
  can_msgs
  dataspeed_ulc_msgs
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Enabling coverage testing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/nodelet.cpp
  src/UlcNode.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(ulc_node
  src/node.cpp
)
target_link_libraries(ulc_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

### Install Rules
install(TARGETS ${PROJECT_NAME} ulc_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_python_setup()
catkin_install_python(PROGRAMS
                        scripts/speed_sine_wave.py
                        scripts/speed_square_wave.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

### Tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
