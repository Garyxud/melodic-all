cmake_minimum_required(VERSION 2.8.3)
project(imagezero_ros)

set(RUNTIME_DEPS
    cv_bridge
    imagezero
    message_runtime
    sensor_msgs
    )

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    ${RUNTIME_DEPS}
    )

# generate the dynamic_reconfigure config file
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${RUNTIME_DEPS}
    DEPENDS OpenCV
    )

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/ros_support.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

