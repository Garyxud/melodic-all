cmake_minimum_required(VERSION 2.8.3)
project(imagezero_image_transport)

set(RUNTIME_DEPS
    cv_bridge
    image_transport
    imagezero_ros
    message_runtime
    sensor_msgs)

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
    src/imagezero_publisher.cpp
    src/imagezero_subscriber.cpp
    src/manifest.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

# add xml file
install(FILES imagezero_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
