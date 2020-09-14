cmake_minimum_required(VERSION 2.8.3)
project(leuze_phidget_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  leuze_msgs
  std_msgs
)

find_package(Boost REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDES}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp leuze_msgs std_msgs
)

add_executable(leuze_phidget_driver_node src/leuze_phidget_driver_node.cpp
                                         src/leuze_phidget_driver.cpp)
target_link_libraries(leuze_phidget_driver_node ${catkin_LIBRARIES})
add_dependencies(leuze_phidget_driver_node ${catkin_EXPORTED_TARGETS})

install(TARGETS leuze_phidget_driver_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
