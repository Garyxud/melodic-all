#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# RosTest
##############################################################################

""" Test loading of interactions to the interactions manager. """

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rostest
import rosunit
import rospy
import rocon_interaction_msgs.msg as interaction_msgs
import rocon_python_comms
import time

##############################################################################
# Imports
##############################################################################

class TestFakeRemocon(unittest.TestCase):
    
    def setUp(self):
        self.number_of_interactive_clients = 0
        rospy.init_node("test_fake_remocon")
        

    def test_fake_remocon(self):
        """ Loading... """
        try:
            topic_name = rocon_python_comms.find_topic('rocon_interaction_msgs/InteractiveClients', timeout=rospy.rostime.Duration(5.0), unique=True)
            rospy.loginfo("TestRemocons : found interactions manager at %s" % topic_name)
        except rocon_python_comms.NotFoundException as e:
            self.fail("couldn't find the interactions topics")
        rospy.Subscriber(topic_name, interaction_msgs.InteractiveClients, self.callback)
        count = 0
        timeout_time = time.time() + 5.0
        while not rospy.is_shutdown() and self.number_of_interactive_clients != 1:
            rospy.rostime.wallsleep(0.1)
            if time.time() > timeout_time:
                break
        self.assertEqual(self.number_of_interactive_clients, 1, 'timed out waiting for publication of our fake remocon')

    def callback(self, msg):
        self.number_of_interactive_clients = len(msg.idle_clients)
        rospy.loginfo("TestRemocons : received interactive client status update [%s]", self.number_of_interactive_clients)
        
    def tearDown(self):
        pass

if __name__ == '__main__':
    rostest.rosrun('rocon_interactions', 'fake_remocon', TestFakeRemocon)
