cmake_minimum_required(VERSION 3.0.2)
project(dataspeed_pds_lcm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dataspeed_pds_msgs
  message_filters
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Enabling coverage testing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

# Find OR install LCM
include(lcm.cmake)

# Generate LCM type header files
FILE(GLOB files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/lcmtypes" "${CMAKE_CURRENT_SOURCE_DIR}/lcmtypes/*.lcm")
FOREACH(filename ${files})
  MESSAGE(STATUS "Generating LCM type header: ${filename}")
  execute_process(
    COMMAND lcm-gen --lazy --cpp --cpp-hpath ${CMAKE_CURRENT_SOURCE_DIR}/include ${filename}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lcmtypes
  )
ENDFOREACH(filename)

catkin_package(
  INCLUDE_DIRS include ${LCM_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${LCM_LIBRARIES}
)

include_directories(
  include
  ${LCM_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/nodelet.cpp
  src/PdsNode.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${LCM_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME pds_node PREFIX "")

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
