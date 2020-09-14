cmake_minimum_required(VERSION 2.8.3)
project(message_relay)

# Add new message packages to process here
set(MESSAGE_PACKAGES
  actionlib_msgs
  controller_manager_msgs
  diagnostic_msgs
  gazebo_msgs
  geometry_msgs
  map_msgs
  move_base_msgs
  multimaster_msgs
  nav_msgs
  robot_localization
  rosgraph_msgs
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  stereo_msgs
  tf2_msgs
  trajectory_msgs
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS roscpp ${MESSAGE_PACKAGES})
find_package(Boost REQUIRED)

# Generate files into devel space's include directory
set(GEN_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
file(MAKE_DIRECTORY ${GEN_DIR}/${PROJECT_NAME}/processor)
file(MAKE_DIRECTORY ${GEN_DIR}/${PROJECT_NAME}/relay_factory)

catkin_package(
  INCLUDE_DIRS include ${GEN_DIR}
  LIBRARIES field_processor message_processor message_relay_factory
  CATKIN_DEPENDS roscpp ${MESSAGE_PACKAGES}
  DEPENDS Boost
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

include_directories(
  include
  ${GEN_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

include(cmake/get_base_names.cmake)

# TODO prevent CMake crosstalk in catkin_make by prepending ${PROJECT_NAME} to all vars
set(GEN_SOURCES)
set(GEN_HEADERS)
set(GEN_TARGETS)
set(ALL_MESSAGE_NAMES)
set(ALL_SERVICE_NAMES)

# Declare templates for message processor generation
set(MESSAGE_PROCESSOR_SOURCE_TMPL ${CMAKE_CURRENT_SOURCE_DIR}/src/processor/package_message_processor.cpp.tmpl)
set(MESSAGE_PROCESSOR_HEADER_TMPL ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/processor/package_message_processor.h.tmpl)

# Iterate over each message package
foreach(MESSAGE_PACKAGE ${MESSAGE_PACKAGES})

  get_base_names(${MESSAGE_PACKAGE}_MESSAGE_FILES "msg" ${MESSAGE_PACKAGE} ${MESSAGE_PACKAGE}_MESSAGE_NAMES )
  list(APPEND ALL_MESSAGE_NAMES ${${MESSAGE_PACKAGE}_MESSAGE_NAMES})

  get_base_names(${MESSAGE_PACKAGE}_SERVICE_FILES "srv" ${MESSAGE_PACKAGE} ${MESSAGE_PACKAGE}_SERVICE_NAMES )
  list(APPEND ALL_SERVICE_NAMES ${${MESSAGE_PACKAGE}_SERVICE_NAMES})

  set(SOURCE_OUTPUT "${GEN_DIR}/${PROJECT_NAME}/processor/${MESSAGE_PACKAGE}_message_processor.cpp")
  set(HEADER_OUTPUT "${GEN_DIR}/${PROJECT_NAME}/processor/${MESSAGE_PACKAGE}_message_processor.h")

  # Generate message processing headers and sources from templates
  add_custom_command(
    OUTPUT ${SOURCE_OUTPUT} ${HEADER_OUTPUT}
    DEPENDS ${MESSAGE_PROCESSOR_SOURCE_TMPL} ${MESSAGE_PROCESSOR_HEADER_TMPL}
    COMMAND ${CATKIN_ENV} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_message_processors.py ${MESSAGE_PACKAGE}
    --msg-names ${${MESSAGE_PACKAGE}_MESSAGE_NAMES}
    --srv-names ${${MESSAGE_PACKAGE}_SERVICE_NAMES}
    --cpp-tmpl ${MESSAGE_PROCESSOR_SOURCE_TMPL}
    --h-tmpl ${MESSAGE_PROCESSOR_HEADER_TMPL}
    --cpp-out ${SOURCE_OUTPUT}
    --h-out ${HEADER_OUTPUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )

  # Intermediate target for generated source and header files, per message package
  set(TARGET_NAME ${MESSAGE_PACKAGE}_message_processor_gen)
  add_custom_target(
    ${TARGET_NAME}
    DEPENDS ${SOURCE_OUTPUT} ${HEADER_OUTPUT}
    SOURCES ${SOURCE_OUTPUT} ${HEADER_OUTPUT}
  )
  add_dependencies(${TARGET_NAME}
    ${MESSAGE_PACKAGE}_generate_messages_cpp
  )

  list(APPEND GEN_SOURCES ${SOURCE_OUTPUT})
  list(APPEND GEN_HEADERS ${HEADER_OUTPUT})
  list(APPEND GEN_TARGETS ${TARGET_NAME})

endforeach()

add_library(field_processor
  src/processor/frame_id_processor.cpp
  src/processor/time_processor.cpp
)
target_link_libraries(field_processor
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Library of all message processors
add_library(message_processor
  ${GEN_SOURCES}
)
add_dependencies(message_processor
  ${GEN_TARGETS}
)
target_link_libraries(message_processor
  field_processor
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

### Factories to create topic/service/action relays

# Generate topic relays
set(TOPIC_RELAY_FACTORY_TMPL ${CMAKE_CURRENT_SOURCE_DIR}/src/relay_factory/topic_relay_factory.cpp.tmpl)
set(TOPIC_RELAY_FACTORY_SOURCE ${GEN_DIR}/${PROJECT_NAME}/relay_factory/topic_relay_factory.cpp)
add_custom_command(
  OUTPUT ${TOPIC_RELAY_FACTORY_SOURCE}
  DEPENDS ${TOPIC_RELAY_FACTORY_TMPL}
  COMMAND ${CATKIN_ENV} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_relay_factory.py
  --msg-srv-names ${ALL_MESSAGE_NAMES}
  --cpp-tmpl ${TOPIC_RELAY_FACTORY_TMPL}
  --cpp-out ${TOPIC_RELAY_FACTORY_SOURCE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts
)

# Generate service relays
set(SERVICE_RELAY_FACTORY_TMPL ${CMAKE_CURRENT_SOURCE_DIR}/src/relay_factory/service_relay_factory.cpp.tmpl)
set(SERVICE_RELAY_FACTORY_SOURCE ${GEN_DIR}/${PROJECT_NAME}/relay_factory/service_relay_factory.cpp)
add_custom_command(
  OUTPUT ${SERVICE_RELAY_FACTORY_SOURCE}
  DEPENDS ${SERVICE_RELAY_FACTORY_TMPL}
  COMMAND ${CATKIN_ENV} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_relay_factory.py
  --msg-srv-names ${ALL_SERVICE_NAMES}
  --cpp-tmpl ${SERVICE_RELAY_FACTORY_TMPL}
  --cpp-out ${SERVICE_RELAY_FACTORY_SOURCE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts
)

# Required to add_dependencies() to library target
add_custom_target(
  ${PROJECT_NAME}_factories
  DEPENDS ${TOPIC_RELAY_FACTORY_SOURCE} ${SERVICE_RELAY_FACTORY_SOURCE}
)

# Library of relay implementations (via header) and factories
add_library(message_relay_factory
  ${TOPIC_RELAY_FACTORY_SOURCE}
  ${SERVICE_RELAY_FACTORY_SOURCE}
  src/relay_factory/action_relay_factory.cpp
)
add_dependencies(message_relay_factory
  ${PROJECT_NAME}_factories
)
target_link_libraries(message_relay_factory
  field_processor
  message_processor
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

### Node to spawn and configure relays

add_executable(message_relay_node
  src/message_relay_node.cpp
)
target_link_libraries(message_relay_node
  field_processor
  message_processor
  message_relay_factory
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS field_processor message_processor message_relay_factory message_relay_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES ${GEN_HEADERS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  roslaunch_add_file_check(launch)
  roslint_cpp()
  roslint_python()
  roslint_add_test()
endif()
