#!/usr/bin/env python
from __future__ import absolute_import

# ROS SETUP if needed

import multiprocessing
import time
import cProfile

import rospy
import rosgraph
import roslaunch
import rocon_python_comms

# Start roslaunch
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

cache_node = rocon_python_comms.ConnectionCache()

def update_loop():
    count = 255
    start = time.time()
    while count > 0:
        # time is ticking
        now = time.time()
        timedelta = now - start
        start = now

        cache_node.update()

        count -= 1

cProfile.run('update_loop()')

rospy.signal_shutdown('test complete')
