cmake_minimum_required(VERSION 3.5)
project(jderobot_camviz)
set(CMAKE_BUILD_TYPE Debug)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
endif()

SET(rosversion "1" CACHE STRING "Some user-specified option")
message("THIS IS ROS1")
SET( SOURCE_FILES camViz.cpp viewer.cpp)
add_definitions(-DGLADE_DIR="${gladedir}")

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-3.0)

#pkg_check_modules(libglademm libglademm-2.4)

if(roscpp_FOUND) 
	message("ROS1 found")
	add_definitions(-DROS1_H) 
	add_definitions(-DROS1S_H)
	SET(ROS1_H ON) 
ELSE()
	message("ROS1 not found")
	SET(ROS1_H OFF) 
	SET(ROS1S_H OFF)
endif()
SET( LIBS_EXTRA resourcelocator )

include_directories(${GTKMM_INCLUDE_DIRS}
    ${resourcelocator_INCLUDE_DIRS}
  # ${libglademm_INCLUDE_DIRS}
  # ${gtkglextmm_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIRS}
)

include_directories(${roscpp_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} )
include_directories(
  ${std_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  #${libglademm_INCLUDE_DIRS}
${resourcelocator_INCLUDE_DIRS})

link_directories(
    ${resourcelocator_LIBRARY_DIRS}
    ${GTKMM_LIBRARY_DIRS}
    ${resourcelocator_LIBRARY_DIRS}

)

add_executable (camViz ${SOURCE_FILES})

target_link_libraries(camViz
${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${image_transport_LIBRARIES}
    ${OpenCV_LIBRARIES} 
    ${cv_bridge_LIBRARIES}
      ${sensor_msgs_LIBRARIES} 
      ${YAML_CPP_LIBRARIES})

TARGET_LINK_LIBRARIES(camViz
    ${GTKMM_LIBRARIES}
  # ${libglademm_LIBRARIES}
    ${ZLIB_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${resourcelocator_LIBRARIES}
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARIES}

    )
install(TARGETS camViz
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
    COMPONENT camviz
    )


INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/camViz.glade DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/glade COMPONENT camviz)
INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/camViz.yml DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf COMPONENT camviz)
