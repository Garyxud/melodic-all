cmake_minimum_required(VERSION 3.0.2)
project(costmap_tf_layer)


find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  pluginlib
  roscpp
  tf2_geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    pluginlib
    roscpp
    tf2_geometry_msgs
    INCLUDE_DIRS include
    LIBRARIES costmap_tf_layer
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/costmap_tf_layer.cpp)
add_dependencies(${PROJECT_NAME} ${costmap_tf_layer_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS costmap_tf_layer
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
