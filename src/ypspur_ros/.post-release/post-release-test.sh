#!/bin/bash

if [ $# -lt 1 ]
then
  echo "$0 ros-distro"
  exit 0
fi

if [ ! -c /dev/ttyACM0 ]
then
  echo "/dev/ttyACM0 not found"
  exit 1
fi

export ROS_DISTRO=$1
echo "===================================="
echo "Testing shadow-fixed for $ROS_DISTRO"
echo "===================================="

set -e

docker build -t ypspur-ros-shadow-fixed:${ROS_DISTRO} --build-arg ROS_DISTRO --build-arg CACHE_CLEAN=`date +%s` .
exec docker-compose up
