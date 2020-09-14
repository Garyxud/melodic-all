cmake_minimum_required(VERSION 2.8.3)
project(sainsmart_relay_usb)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/RelayNode.cpp src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ftdi)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME relay_node PREFIX "")

add_executable(${PROJECT_NAME}_list src/list.cpp)
target_link_libraries(${PROJECT_NAME}_list ${catkin_LIBRARIES} ftdi)
set_target_properties(${PROJECT_NAME}_list PROPERTIES OUTPUT_NAME list PREFIX "")

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_list
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_BUILD_BINARY_PACKAGE)
  # Install udev rules with dh_installudev: http://manpages.ubuntu.com/manpages/man1/dh_installudev.1.html
  STRING(REPLACE "_" "-" DEBIAN_PACKAGE_NAME ros-$ENV{ROS_DISTRO}-${PROJECT_NAME})
  FILE(REMOVE ${CMAKE_CURRENT_SOURCE_DIR}/debian/${DEBIAN_PACKAGE_NAME}.udev)
  FILE(GLOB UDEV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/udev" "${CMAKE_CURRENT_SOURCE_DIR}/udev/*.rules")
  FOREACH(FILENAME ${UDEV_FILES})
    MESSAGE(STATUS "Appending contents of udev/${FILENAME} to debian/${DEBIAN_PACKAGE_NAME}.udev")
    FILE(READ udev/${FILENAME} FILEDATA)
    FILE(APPEND ${CMAKE_CURRENT_SOURCE_DIR}/debian/${DEBIAN_PACKAGE_NAME}.udev ${FILEDATA})
  ENDFOREACH(FILENAME)

  # ROS Indigo/Jade on Ubuntu versions earlier than Xenial default to a udev priority of 40, which is not enough to override the default usb rule of priority 50
  # http://manpages.ubuntu.com/manpages/trusty/man1/dh_installudev.1.html
  # For ROS versions earlier than Kinetic, install duplicate rule of priority 59
  if($ENV{ROS_DISTRO} STRLESS kinetic)
    install(FILES debian/${DEBIAN_PACKAGE_NAME}.udev
            DESTINATION /lib/udev/rules.d
            RENAME 59-${DEBIAN_PACKAGE_NAME}.rules
    )
  endif()
else()
  install(DIRECTORY udev
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

