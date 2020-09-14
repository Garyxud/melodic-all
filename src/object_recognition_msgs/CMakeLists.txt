cmake_minimum_required(VERSION 2.8)
project(object_recognition_msgs)

# generate the messages and the actionlib server for ROS
find_package(catkin REQUIRED actionlib_msgs genmsg geometry_msgs sensor_msgs shape_msgs std_msgs)

add_action_files(DIRECTORY action FILES ObjectRecognition.action)
add_service_files(DIRECTORY srv FILES GetObjectInformation.srv)

add_message_files(DIRECTORY msg
                  FILES ObjectType.msg
                        ObjectInformation.msg
                        RecognizedObject.msg
                        RecognizedObjectArray.msg
                        Table.msg
                        TableArray.msg
)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs sensor_msgs shape_msgs std_msgs)

# define the project
catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs sensor_msgs shape_msgs std_msgs)
