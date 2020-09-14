cmake_minimum_required(VERSION 2.8.3)
project(leuze_rsl_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  leuze_msgs
  angles
)

find_package(Boost REQUIRED)


include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDES}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs leuze_msgs
)

add_executable(leuze_rsl_driver src/leuze_rsl_driver.cpp
                    src/rsl400/rsl400_interface.cpp)
target_link_libraries(leuze_rsl_driver ${catkin_LIBRARIES})
add_dependencies(leuze_rsl_driver ${catkin_EXPORTED_TARGETS})


install(TARGETS leuze_rsl_driver RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(test_parse_data test/test_parse_data.test test/test_parse_data.cpp src/rsl400/rsl400_interface.cpp)
    target_link_libraries(test_parse_data ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(test_parse_data ${catkin_EXPORTED_TARGETS})
endif()
