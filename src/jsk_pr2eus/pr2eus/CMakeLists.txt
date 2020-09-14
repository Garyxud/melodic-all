cmake_minimum_required(VERSION 2.8.3)
project(pr2eus)

find_package(pr2_controllers_msgs QUIET) # pr2_controllers_msgs is not releaased on jade/kinetic...
if(pr2_controllers_msgs_FOUND)
  message(STATUS "pr2_controllers_msgs FOUND")
  set(PR2_CONTROLLERS_MSGS_PACKAGE pr2_controllers_msgs)
else()
  message(STATUS "pr2_controllers_msgs IS DEPRECATED")
  set(PR2_CONTROLLERS_MSGS_PACKAGE )
endif()
find_package(pr2_mechanism_msgs QUIET)
if(pr2_mechanism_msgs_FOUND)
  message(STATUS "pr2_mechanism_msgs FOUND")
  set(PR2_MECHANISM_MSGS_PACKAGE pr2_mechanism_msgs)
else()
  message(STATUS "pr2_mechanism_msgs NOT FOUND")
  set(PR2_MECHANISM_MSGS_PACKAGE )
endif()
if($ENV{ROS_DISTRO} STREQUAL "hydro")
  set(GENEUS_PACKAGE geneus)
endif()

find_package(catkin REQUIRED COMPONENTS euscollada control_msgs nav_msgs dynamic_reconfigure rosgraph_msgs move_base_msgs pr2_msgs ${GENEUS_PACKAGE} ${PR2_CONTROLLERS_MSGS_PACKAGE} ${PR2_MECHANISM_MSGS_PACKAGE} sound_play
  roseus # this load roseus.cmake, so it needs to be located in the end
  )


catkin_package()

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

file(GLOB _install_files RELATIVE ${PROJECT_SOURCE_DIR} *.l *.launch)
install(FILES ${_install_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES pr2-send-joints.l pr2-read-state.l
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PERMISSIONS WORLD_EXECUTE
  )

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  if(pr2_controllers_msgs_FOUND)
    add_rostest(test/pr2eus-test.launch)
    add_rostest(test/make-pr2-model-file-test.launch)
    find_package(pr2_gazebo QUIET)
    if(pr2_gazebo_FOUND AND (NOT pr2_gazebo_VERSION VERSION_LESS 2.0.7)) # test only supports indigo
      add_rostest(test/pr2-ri-test.launch)
    endif()
    add_rostest(test/pr2-ri-test-simple.launch)
    add_rostest(test/robot-init-test.test) # this uses pr2
  endif()
  add_rostest(test/robot-no-clock.test)
  add_rostest(test/default-ri-test.test)
  add_rostest(test/speak-test.test)
  add_rostest(test/speak-test.test ARGS test_action:=true)
  add_rostest(test/speak-test-eus.test)
  add_rostest(test/speak-test-eus.test ARGS test_action:=true)
endif()

generate_eusdoc(robot-interface.l)
generate_eusdoc(pr2-interface.l)
generate_eusdoc(pr2-utils.l)
generate_eusdoc(speak.l)
