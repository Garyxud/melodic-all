
cmake_minimum_required(VERSION 2.8.3)
project(jderobot_carviz)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED)
catkin_package()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
endif()


SET(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH} ")

	SET(SOURCE main.cpp  
				    robot/actuators.cpp robot/robot.cpp robot/sensors.cpp  
			        gui/gui.cpp gui/threadupdategui.cpp
			        gui/widget/cameraswidget.cpp gui/widget/controlvw.cpp gui/widget/laserwidget.cpp 
              config/loader.cpp config/properties.cpp
              comm/cameraClient.cpp comm/communicator.cpp comm/laserClient.cpp comm/motorsClient.cpp comm/pose3dClient.cpp
              comm/src/listenerCamera.cpp comm/src/listenerLaser.cpp comm/src/listenerPose.cpp comm/src/publisherMotors.cpp comm/src/translators.cpp
			        ${HEADERS}
			        ${RESOURCES}
    )

find_package(Qt5 REQUIRED Widgets)

        find_package(roscpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(image_transport REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(OpenCV REQUIRED)
        find_package(cv_bridge REQUIRED)
        find_package(Boost 1.65 REQUIRED COMPONENTS system)

        find_package(yaml-cpp REQUIRED)

if(roscpp_FOUND) 
message("ROS1 found")
add_definitions(-DROS_H) 
add_definitions(-DROS1S_H)
SET(ROS_H ON) 
ELSE()
message("ROS1 not found")
SET(ROS_H OFF) 
SET(ROS1S_H OFF)
endif()

#if (${QT5_COMPILE} AND ${roscpp_FOUND})
	
    set(GUI_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/gui)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} ${Qt5Core_EXECUTABLE_COMPILE_FLAGS} ${Qt5Gui_EXECUTABLE_COMPILE_FLAGS} ${Qt5OpenGL_EXECUTABLE_COMPILE_FLAGS}")


    add_definitions(${Qt5Widgets_DEFINITIONS}
                        ${Qt5Core_DEFINITIONS}
                        ${Qt5Gui_DEFINITIONS}
                        ${Qt5OpenGL_DEFINITIONS}
    )


    SET(HEADERS robot/robot.h gui/gui.h gui/widget/cameraswidget.h gui/widget/controlvw.h)
    SET(RESOURCES resources.qrc)

include_directories(${roscpp_INCLUDE_DIRS}   ${std_msgs_INCLUDE_DIRS}
 ${image_transport_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} )


    include_directories(
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        #${Ice_INCLUDE_DIR}
        ${INTERFACES_CPP_DIR}
        ${LIBS_DIR}/
        ${Boost_INCLUDE_DIRS}

        ${CMAKE_CURRENT_SOURCE_DIR}
        ${config_INCLUDE_DIRS}
        ${jderobottypes_INCLUDE_DIRS}
        ${carviz_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${ZLIB_INCLUDE_DIRS}
    )

	include_directories(carviz)
	include_directories(config)
	

    link_directories(${JDE_LIBS}
        ${easyiceconfig_LIBRARY_DIRS}
        ${carviz_LIBRARY_DIRS}
    )

    add_executable( carViz ${SOURCE} ${INTERFACES_CPP_DIR})
			       
    target_link_libraries(carViz Qt5::Widgets
        ${Qt5Widgets_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}

 ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
   ${image_transport_LIBRARIES}
     ${OpenCV_LIBRARIES} 
     ${cv_bridge_LIBRARIES}
      ${sensor_msgs_LIBRARIES}

        ${Qt5Core_LIBRARIES}
        ${Qt5Gui_LIBRARIES}
        ${Qt5OpenGL_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        ${OpenCV_LIBRARIES}
        ${config_LIBRARIES}
        ${carviz_LIBRARIES}
        ${ZeroCIce_LIBRARIES}
 
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${ZLIB_LIBRARIES}
       
            )
    install(TARGETS carViz
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
    COMPONENT carviz
    )

    INSTALL (FILES ${CMAKE_CURRENT_SOURCE_DIR}/carViz.yml 
		DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jderobot/conf 
		COMPONENT carviz)
#ENDIF()
