cmake_minimum_required(VERSION 2.8.3)
project(kvh_geo_fog_3d_msgs)

# Find what is required to build
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

# Add our message files
add_message_files(DIRECTORY msg)

# Generate our messages
generate_messages(DEPENDENCIES std_msgs)

# Add our messages to the package
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
