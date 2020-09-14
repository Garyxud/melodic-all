#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash

roscore &
pid_roscore=$!
sleep 3

rosrun ypspur_ros ypspur_ros _param_file:=/icart-mini.param _device:=/dev/ttyACM0 &
pid=$!

sleep 10

kill -SIGINT ${pid}
wait ${pid}
kill -SIGINT ${pid_roscore}
wait ${pid_roscore}
