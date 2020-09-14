cmake_minimum_required(VERSION 2.8.3)
project(asr_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	message_generation
	geometry_msgs
	std_msgs
)

add_message_files(DIRECTORY msg FILES 
	AsrObject.msg 
	AsrTypeAndId.msg
	AsrGlove.msg 
	AsrAttributedPointCloud.msg
	AsrAttributedPoint.msg
	AsrViewport.msg
)

generate_messages(DEPENDENCIES
	geometry_msgs
	std_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
