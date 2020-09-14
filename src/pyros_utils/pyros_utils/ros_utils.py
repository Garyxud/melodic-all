#!/usr/bin/env python
# A very special ROS hack that emulate a ros environment when imported from python
# Useful for using all python tools ( tests, IDE, etc. ) without having to do all the ROS setup beforehand
from __future__ import absolute_import

import os
import multiprocessing
import time

# Note : ROS setup must be done before importing this ( by calling ros_setup.ROS_emulate_setup() )
import rosgraph
import roslaunch


def get_master(spawn=True):
    """
    Returns an instance of Master to access it via its API
    If needed starts roscore.
    :return:
    """
    master = rosgraph.Master('pyros_utils')
    roscore_process = None
    if not master.is_online() and spawn:
        # Trying to solve this : http://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/
        def ros_core_launch():
            roslaunch.main(['roscore', '--core'])  # same as rostest_main implementation

        roscore_process = multiprocessing.Process(target=ros_core_launch)
        roscore_process.start()

        # waiting for master to come up
        while not master.is_online():
            time.sleep(1)

    return master, roscore_process


# this is already defined in rospkg...
def get_ros_home():
    return os.environ.get('ROS_HOME', os.path.join(os.environ['HOME'], '.ros'))
