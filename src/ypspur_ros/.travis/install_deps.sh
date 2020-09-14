#!/bin/bash

set -o errexit
set -o verbose

scriptdir=`dirname "${0}"`

source /opt/ros/${ROS_DISTRO}/setup.bash
source /catkin_ws/devel/setup.bash

cd /catkin_ws

SKIP_KEYS=
if [ -f $scriptdir/rosdepkeys.skip ];
then
  SKIP_KEYS="--skip-keys=\"$(cat $scriptdir/rosdepkeys.skip | tr '\n' ' ' | sed 's/\s*$//g')\""
fi

echo $SKIP_KEYS

apt-get -qq update
eval rosdep install --from-paths src --ignore-src $SKIP_KEYS --rosdistro=${ROS_DISTRO} -y
apt-get clean
rm -rf /var/lib/apt/lists/*
