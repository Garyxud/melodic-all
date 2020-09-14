cmake_minimum_required(VERSION 2.8)
project(cob_hand_bridge)

find_package(Boost COMPONENTS chrono system thread REQUIRED)
include_directories(pigpio ros_lib ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

add_library(cob_hand_ros_lib STATIC ros_lib/duration.cpp ros_lib/time.cpp)

add_library(cob_hand_pigpio STATIC pigpio/pigpio.c pigpio/command.c)
target_compile_definitions(cob_hand_pigpio PRIVATE DISABLE_SER_CHECK_INITED) # for tests

add_executable(cob_hand_bridge src/bridge.cpp)
target_link_libraries( cob_hand_bridge cob_hand_pigpio cob_hand_ros_lib ${Boost_LIBRARIES} )

install(TARGETS cob_hand_bridge DESTINATION bin)
