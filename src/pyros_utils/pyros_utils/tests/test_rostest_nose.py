#!/usr/bin/env python
from __future__ import absolute_import

# Adding current package repository in order to be able to import it (if started with python cli)
import sys
import os
current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# if not current_path in sys.path:  # this gets in the way with ROs emulated setup
sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec
import time

# importing current package
import pyros_utils

# importing ros stuff
import roslaunch
import rospy
import rosnode

# importing unittest and nose
import unittest
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


@nose.tools.nottest
def main_test_part(rostest_nose, roslaunch, rospy, rosnode):
    print("Entering main_test_part")

    # if running with rostest we cannot check the startup behavior from inside the test
    if not rostest_nose.is_rostest_enabled():
        rostest_nose.rostest_nose_setup_module()

        # Start roslaunch
        # Ref : http://wiki.ros.org/roslaunch/API%20Usage
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        rospy.set_param('/string_pub_node/topic_name', '~test_str_topic')  # private topic name to not mess things up too much
        rospy.set_param('/string_pub_node/test_message', 'testing topic discovery')
        package = 'pyros_test'
        executable = 'string_pub_node.py'
        name = 'string_pub_node'
        node = roslaunch.core.Node(package, executable, name)

        talker_process = launch.launch(node)
        assert talker_process.is_alive()

    try:
        # wait for the node to come up (needed for both rostest and nosetest)
        with timeout(5) as t:
            while not t.timed_out and not rosnode.rosnode_ping("string_pub_node", max_count=1):
                time.sleep(1)
        # TODO: improve this

        # try a few times
        assert rosnode.rosnode_ping("string_pub_node", max_count=5)

    # we make sure this always run to avoid hanging with child processes
    finally:
        if not rostest_nose.is_rostest_enabled():
            talker_process.stop()
            assert not talker_process.is_alive()

            # if running with rostest, we cannot check the shutdown behavior form inside the test
            # so we do it here
            assert not rosnode.rosnode_ping("string_pub_node", max_count=5)

            rostest_nose.rostest_nose_teardown_module()

            rospy.signal_shutdown('test complete')

    print("Exiting main_test_part")


# this test is used to validate pyros_utils/rostest_nose behavior if imported after delayed import
def test_import_after_delay_setup_teardown_module():
    print("Checking importing rostest_nose after delayed_import call")
    # setup_module part
    from pyros_utils import rostest_nose
    main_test_part(rostest_nose, roslaunch, rospy, rosnode)


# this test is used to validate pyros_utils/rostest_nose behavior if imported before delayed import
def test_import_before_delay_setup_teardown_module():
    print("Checking importing rostest_nose before delayed_import call")
    # setup_module part
    from pyros_utils import rostest_nose
    main_test_part(rostest_nose, roslaunch, rospy, rosnode)
    

# this test is only to validate pyros_utils/rostest_nose compatibility with rostest
# It is written following rostest style
@nose.tools.nottest
class testRosTest(unittest.TestCase):
    def test_rostest_detection(self):
        # check that pyros_utils/rostest_nose is usable even without delay import magic
        from pyros_utils import rostest_nose

        print("Checking rostest detection...")
        # checking detection works properly
        assert rostest_nose.is_rostest_enabled()
        # checking setup is done as expected                

        # wait for the node to come up
        with timeout(5) as t:
            while not t.timed_out and not rosnode.rosnode_ping("string_pub_node", max_count=1):
                time.sleep(1)
        # TODO: improve this

        assert rosnode.rosnode_ping("string_pub_node", max_count=5)
        # note we cannot test cleanup here. Assuming rostest does it properly.

    # Tests Needed to validate that tests written for nose with pyros can also work with rostest
    # if called from usual rostest testCase
    def test_import_after_delay_setup_teardown_module(self):
        test_import_after_delay_setup_teardown_module()

    def test_import_before_delay_setup_teardown_module(self):
        test_import_before_delay_setup_teardown_module()


if __name__ == '__main__':
    print("Running with ROStest if launched with rostest...")
    # testing rostest to validate rostests will still work.
    from pyros_utils import rostest_nose
    rostest_nose.rostest_or_nose_main('pyros_utils', 'testRosTest', testRosTest, sys.argv)
            
    if rostest_nose.is_rostest_enabled():
        print("Force running with nose even if launched with rostest...")
        # Needed to validate that rostest_nose.rostest_or_nose_main will work fine if calling nose

        # we need to remove options that nose doesnt know
        #print(sys.argv)
        for opt in sys.argv[1:]:  # we keep argv[0]
            if opt.startswith("--gtest_output") or opt.startswith("__name") or opt.startswith("__log") or opt.startswith("--text"):
                sys.argv.remove(opt)
        #print(sys.argv)

        rostest_nose.rostest_enabled = False

        # as well as nose
        nose.runmodule()

    
    
    
