#!/usr/bin/env python
from __future__ import absolute_import

# Adding current package repository in order to be able to import it (if started with python cli)
import sys
import os
current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# if not current_path in sys.path:  # this gets in the way with ROs emulated setup
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec
import time

import rospy
import rosnode
import roslaunch

# Import current pacakge
import pyros_utils

# importing nose
import nose


class timeout(object):
    """
    Small useful timeout class
    """
    def __init__(self, seconds):
        self.seconds = seconds

    def __enter__(self):
        self.die_after = time.time() + self.seconds
        return self

    def __exit__(self, type, value, traceback):
        pass

    @property
    def timed_out(self):
        return time.time() > self.die_after


def test_roscore_started():
    master, roscore = pyros_utils.get_master()
    try:
        assert master.is_online()
    finally:  # to make sure we always shutdown everything, even if test fails
        if roscore is not None:
            roscore.terminate()
        rospy.signal_shutdown('test_roscore_started done')
        while roscore and roscore.is_alive():
            time.sleep(0.2)  # waiting for roscore to die
        assert not (roscore and master.is_online())


def test_roslaunch_started():
    master, roscore = pyros_utils.get_master()
    try:
        assert master.is_online()

        # hack needed to wait for master until fix for https://github.com/ros/ros_comm/pull/711 is released
        roslaunch.rlutil.get_or_generate_uuid(None, True)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        assert launch.started

        launch.stop()

    finally:  # to make sure we always shutdown everything, even if test fails
        if roscore is not None:
            roscore.terminate()
        rospy.signal_shutdown('test_roslaunch_started done')
        while roscore and roscore.is_alive():
            time.sleep(0.2)  # waiting for roscore to die
        assert not (roscore and master.is_online())


def test_rosnode_started():
    master, roscore = pyros_utils.get_master()
    try:
        assert master.is_online()

        # hack needed to wait for master until fix for https://github.com/ros/ros_comm/pull/711 is released
        roslaunch.rlutil.get_or_generate_uuid(None, True)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        assert launch.started

        echo_node = roslaunch.core.Node('pyros_test', 'echo.py', name='echo')
        echo_process = launch.launch(echo_node)
        assert echo_process.is_alive()

        node_api = None
        with timeout(5) as t:
            while not t.timed_out and node_api is None:
                node_api = rosnode.get_api_uri(master, 'echo')  # would be good to find a way to do this without rosnode dependency
        assert node_api is not None

        launch.stop()

    finally:  # to make sure we always shutdown everything, even if test fails
        if roscore is not None:
            roscore.terminate()
        rospy.signal_shutdown('test_rosnode_started done')
        while roscore and roscore.is_alive():
            time.sleep(0.2)  # waiting for roscore to die
        assert not (roscore and master.is_online())

if __name__ == '__main__':
    # forcing nose run from python call
    nose.runmodule()
