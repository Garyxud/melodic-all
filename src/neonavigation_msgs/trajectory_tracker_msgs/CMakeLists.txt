cmake_minimum_required(VERSION 3.0.2)
project(trajectory_tracker_msgs)

set(MESSAGE_DEPENDS
  geometry_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${MESSAGE_DEPENDS})

add_service_files(
  FILES
    ChangePath.srv
)
add_message_files(
  FILES
    PathWithVelocity.msg
    PoseStampedWithVelocity.msg
    TrajectoryServerStatus.msg
    TrajectoryTrackerStatus.msg
)
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDS})
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)

  find_package(roscpp REQUIRED)
  find_package(rosunit REQUIRED)

  add_compile_options(-std=c++11)

  catkin_add_gtest(test_convert test/src/test_convert.cpp)
  target_link_libraries(test_convert ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
