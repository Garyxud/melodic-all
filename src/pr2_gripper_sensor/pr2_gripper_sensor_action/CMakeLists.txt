# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_gripper_sensor_action)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib pr2_gripper_sensor_msgs pr2_mechanism_controllers robot_mechanism_controllers pr2_controllers_msgs pr2_mechanism_model pr2_gripper_sensor_controller pr2_machine message_generation)

catkin_package(
    DEPENDS roscpp actionlib_msgs actionlib pr2_gripper_sensor_msgs pr2_mechanism_controllers robot_mechanism_controllers pr2_controllers_msgs pr2_mechanism_model pr2_gripper_sensor_controller pr2_machine message_runtime
    CATKIN_DEPENDS  # TODO
    INCLUDE_DIRS  ${catkin_INCLUDE_DIRS}# TODO include
    LIBRARIES # TODO
)

include_directories(${catkin_INCLUDE_DIRS})

link_directories(${catkin_INCLUDE_DIRS})

add_executable(pr2_gripper_sensor_action src/pr2_gripper_sensor_action.cpp)
add_dependencies(pr2_gripper_sensor_action ${catkin_EXPORTED_TARGETS})
target_link_libraries(pr2_gripper_sensor_action ${catkin_LIBRARIES})

add_executable(pr2_gripper_findContact_action src/pr2_gripper_findContact_action.cpp)
target_link_libraries(pr2_gripper_findContact_action ${catkin_LIBRARIES})
add_dependencies(pr2_gripper_findContact_action ${catkin_EXPORTED_TARGETS})

add_executable(pr2_gripper_slipServo_action src/pr2_gripper_slipServo_action.cpp)
target_link_libraries(pr2_gripper_slipServo_action ${catkin_LIBRARIES})
add_dependencies(pr2_gripper_slipServo_action ${catkin_EXPORTED_TARGETS})

add_executable(pr2_gripper_event_detector_action     src/pr2_gripper_event_detector_action.cpp)
target_link_libraries(pr2_gripper_event_detector_action ${catkin_LIBRARIES})
add_dependencies(pr2_gripper_event_detector_action ${catkin_EXPORTED_TARGETS})

add_executable(pr2_gripper_forceServo_action     src/pr2_gripper_forceServo_action.cpp)
target_link_libraries(pr2_gripper_forceServo_action ${catkin_LIBRARIES})
add_dependencies(pr2_gripper_forceServo_action pr2_gripper_sensor_msgs_gencpp)

add_executable(positionActionTest              src/positionActionTest.cpp)
target_link_libraries(positionActionTest ${catkin_LIBRARIES})

add_executable(findContactActionTest          src/findContactActionTest.cpp)
add_dependencies(findContactActionTest ${catkin_EXPORTED_TARGETS})
target_link_libraries(findContactActionTest ${catkin_LIBRARIES})

add_executable(slipServoActionTest          src/slipServoActionTest.cpp)
target_link_libraries(slipServoActionTest ${catkin_LIBRARIES})
add_dependencies(slipServoActionTest ${catkin_EXPORTED_TARGETS})

add_executable(placeActionTest          src/placeActionTest.cpp)
target_link_libraries(placeActionTest ${catkin_LIBRARIES})
add_dependencies(placeActionTest ${catkin_EXPORTED_TARGETS})

add_executable(forceServoActionTest          src/forceServoActionTest.cpp)
target_link_libraries(forceServoActionTest ${catkin_LIBRARIES})
add_dependencies(forceServoActionTest ${catkin_EXPORTED_TARGETS})

add_executable(pr2_gripper_grab_action          src/pr2_gripper_grab_action.cpp)
target_link_libraries(pr2_gripper_grab_action ${catkin_LIBRARIES})
add_dependencies(pr2_gripper_grab_action ${catkin_EXPORTED_TARGETS})

add_executable(pr2_gripper_release_action          src/pr2_gripper_release_action.cpp)
target_link_libraries(pr2_gripper_release_action ${catkin_LIBRARIES})
add_dependencies(pr2_gripper_release_action ${catkin_EXPORTED_TARGETS})

add_executable(grabActionTest          src/grabActionTest.cpp)
target_link_libraries(grabActionTest ${catkin_LIBRARIES})
add_dependencies(grabActionTest ${catkin_EXPORTED_TARGETS})

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  pr2_gripper_sensor_action
  pr2_gripper_findContact_action
  pr2_gripper_slipServo_action
  pr2_gripper_event_detector_action
  pr2_gripper_forceServo_action
  pr2_gripper_grab_action
  pr2_gripper_release_action
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY scripts launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
