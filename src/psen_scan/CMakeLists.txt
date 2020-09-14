# Copyright (c) 2019 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8.12)
project(psen_scan)

add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

###############
## Sanitizer ##
###############
# to run (only one sanitizer at a time):
# 1. Set environment variables for log_path in your shell
# export ASAN_OPTIONS="log_path=/path/to/asan-logs"
# export TSAN_OPTIONS="log_path=/path/to/tsan-logs"
# export UBSAN_OPTIONS="log_path=/path/to/ubsan-logs"
# 2. Compile
# catkin_make -DCATKIN_SANITIZER=address|thread|undefined
# 3. Run the node
# 4. Check log_path folder for contents
# build and devel folders have to be deleted before run
if(CATKIN_SANITIZER)
  set(SANITIZER_OPTIONS "address|thread|undefined")
  if(CATKIN_SANITIZER MATCHES ${SANITIZER_OPTIONS})
    message(STATUS "${CATKIN_SANITIZER}-sanitizer activated.")
    add_compile_options(-fsanitize=${CATKIN_SANITIZER} -g)
  else()
    message(FATAL_ERROR "${CATKIN_SANITIZER} is not a valid sanitizer. Valid options are: [${SANITIZER_OPTIONS}]")
  endif()
endif()

################
## Clang tidy ##
################
# to run: catkin_make -DCATKIN_ENABLE_CLANG_TIDY=true
# build and devel folders have to be deleted before run
if(CATKIN_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  endif()
endif()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_sources
  src/scanner.cpp
  src/scanner_frames.cpp
  src/laserscan.cpp
  src/psen_scan_internal_angle.cpp
  src/psen_scan_udp_interface.cpp
)

set(${PROJECT_NAME}_node_sources
  src/psen_scan_driver.cpp
  src/ros_parameter_handler.cpp
  src/ros_scanner_node.cpp
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_node_sources})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############
##  Test  ##
############
if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(pilz_testutils REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

  roslaunch_add_file_check(launch DEPENDENCIES ${PROJECT_NAME}_node)

  include_directories(include
    test/include
    ${pilz_testutils_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIR}
  )

  ##################
  ##  Unit-Tests  ##
  ##################
  catkin_add_gmock(unittest_scanner
    test/unit_tests/unittest_scanner.cpp
    src/scanner.cpp
    src/scanner_frames.cpp
    src/laserscan.cpp
    src/psen_scan_internal_angle.cpp
    src/psen_scan_udp_interface.cpp
  )
  target_link_libraries(unittest_scanner
    ${catkin_LIBRARIES}
    gtest_main
  )

  catkin_add_gtest(unittest_psen_scan_internal_angle
    test/unit_tests/unittest_psen_scan_internal_angle.cpp
    src/psen_scan_internal_angle.cpp
  )
  target_link_libraries(unittest_psen_scan_internal_angle
    ${catkin_LIBRARIES}
    gtest_main
  )

  catkin_add_gtest(unittest_decrypt_password
    test/unit_tests/unittest_decrypt_password.cpp
    src/ros_parameter_handler.cpp
    src/psen_scan_internal_angle.cpp
  )
  target_link_libraries(unittest_decrypt_password
    ${catkin_LIBRARIES}
    gtest_main
  )

  catkin_add_gtest(unittest_build_new_exceptions
    test/unit_tests/unittest_new_build_ros_message_exception.cpp
    test/unit_tests/unittest_new_coherent_monitoring_frames_exception.cpp
    test/unit_tests/unittest_new_decrypt_password_exception.cpp
    test/unit_tests/unittest_new_diagnostic_information_exception.cpp
    test/unit_tests/unittest_new_fetch_monitoring_frame_exception.cpp
    test/unit_tests/unittest_new_get_ros_parameter_exception.cpp
    test/unit_tests/unittest_new_parse_monitoring_frame_exception.cpp
    test/unit_tests/unittest_new_psen_scan_fatal_exception.cpp
    test/unit_tests/unittest_new_udp_read_timeout_exception.cpp
  )
  target_link_libraries(unittest_build_new_exceptions
    ${catkin_LIBRARIES}
    gtest_main
  )

  catkin_add_gtest(unittest_timeout_adjust_func
    test/unit_tests/unittest_timeout_adjust_func.cpp
  )
  target_link_libraries(unittest_timeout_adjust_func
    ${catkin_LIBRARIES}
    gtest_main
  )

  catkin_add_gtest(unittest_scanner_communication_interface
    test/unit_tests/unittest_scanner_communication_interface.cpp
    src/psen_scan_udp_interface.cpp
    src/psen_scan_internal_angle.cpp
  )
  target_link_libraries(unittest_scanner_communication_interface
    ${catkin_LIBRARIES}
    gtest_main
  )

  #########################
  ##  Integration-Tests  ##
  #########################
  catkin_add_gmock(integrationtest_psen_scan_udp_interface
    test/integration_tests/integrationtest_psen_scan_udp_interface.cpp
    src/psen_scan_internal_angle.cpp
    src/psen_scan_udp_interface.cpp
  )
  target_link_libraries(integrationtest_psen_scan_udp_interface
    ${catkin_LIBRARIES}
    ${pilz_testutils_LIBRARIES}
    gtest_main
  )

  catkin_add_executable_with_gmock(${PROJECT_NAME}_integrationtest_scan_topic_node
    test/integration_tests/integrationtest_scan_topic_node.cpp
    src/laserscan.cpp
    src/psen_scan_internal_angle.cpp
    src/ros_scanner_node.cpp
  )
  target_link_libraries(${PROJECT_NAME}_integrationtest_scan_topic_node
    ${catkin_LIBRARIES}
  )

  add_rostest(
    test/integration_tests/integrationtest_scan_topic.test DEPENDENCIES ${PROJECT_NAME}_integrationtest_scan_topic_node
  )

  add_rostest_gtest(integrationtest_ros_parameter_handler
    test/integration_tests/integrationtest_ros_parameter_handler.test
    test/integration_tests/integrationtest_ros_parameter_handler.cpp
    src/ros_parameter_handler.cpp
    src/psen_scan_internal_angle.cpp
  )
  target_link_libraries(integrationtest_ros_parameter_handler
    ${catkin_LIBRARIES}
  )

  add_rostest_gmock(integrationtest_ros_scanner_node
    test/integration_tests/integrationtest_ros_scanner_node.test
    test/integration_tests/integrationtest_ros_scanner_node.cpp
    src/ros_scanner_node.cpp
    src/laserscan.cpp
    src/psen_scan_internal_angle.cpp
  )
  target_link_libraries(integrationtest_ros_scanner_node
    ${catkin_LIBRARIES}
  )

  # to run: catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug psen_scan_coverage (adding -j1 recommended)
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*"
                          "*/${PROJECT_NAME}/src/psen_scan_driver.cpp")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
    )
  endif()
endif()
