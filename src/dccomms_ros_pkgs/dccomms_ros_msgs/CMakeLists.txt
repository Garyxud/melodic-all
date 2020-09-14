cmake_minimum_required(VERSION 2.8.3)
project(dccomms_ros_msgs)

find_package(catkin REQUIRED
        roscpp
        rospy
	std_msgs
	std_srvs
	message_generation
	actionlib
	actionlib_msgs
)

 add_service_files(
   DIRECTORY srv
   FILES
   AddAcousticDevice.srv
   CheckDevice.srv
   CheckChannel.srv
   RemoveDevice.srv
   AddAcousticChannel.srv
   LinkDeviceToChannel.srv
   StartSimulation.srv
   AddCustomDevice.srv
   AddCustomChannel.srv
 )

 generate_messages(
   DEPENDENCIES
   roscpp
   std_msgs
   actionlib_msgs
 )


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS
       roscpp
       std_msgs
       actionlib_msgs
       message_runtime
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
