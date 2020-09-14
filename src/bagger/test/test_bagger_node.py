#!/usr/bin/env python

PKG = 'bagger'
NAME = 'test_bagger_node'

import unittest
import rosunit
import rospy
import time
import subprocess
from random import *
from bagger.msg import BaggingState
from bagger.srv import SetBagState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class BagStateCallback(object):
    def __init__(self):
        self.received_msgs = 0
        self.last_msg = None

    def __call__(self, msg):
        self.received_msgs += 1
        self.last_msg = msg
        self.bag_names_states = {}
        for i in range(0, len(self.last_msg.bag_profile_names)):
            self.bag_names_states[msg.bag_profile_names[i]] = msg.bagging[i]

class TestBaggerNode(unittest.TestCase):
    def __init__(self, *args):
        super(TestBaggerNode, self).__init__(*args)
        self.node = rospy.init_node(NAME)

    def test_bagging_state_names_outputs(self):
        """Test that bagging state messages are correctly published"""

        # Create callback handler and subscribe
        bagging_state_cb = BagStateCallback()
        rospy.Subscriber('/bagger/bag_states', BaggingState, bagging_state_cb)
        set_bagging_state_srv = rospy.ServiceProxy('/bagger/set_bag_state', SetBagState)
        
        # Sleep a little and assert that both bag profiles are in the off state
        time.sleep(0.5)
        self.assertEquals(bagging_state_cb.bag_names_states["everything"], False)
        self.assertEquals(bagging_state_cb.bag_names_states["nav"], False)

        # Set bag states to false - reset state
        self.assertTrue(set_bagging_state_srv("everything", False))
        self.assertTrue(set_bagging_state_srv("nav", False))
        
    def test_basic_set_bag_states(self):
        """Test turning on/off a couple of bags and seeing that their status is updated correctly"""

        # Create callback handler
        set_bagging_state_srv = rospy.ServiceProxy('/bagger/set_bag_state', SetBagState)
        
        # Now that the states are known, subscribe and setup cb
        bagging_state_cb = BagStateCallback()
        rospy.Subscriber('/bagger/bag_states', BaggingState, bagging_state_cb)
        
        time.sleep(0.5)

        # Turn on the everything bag and the navigation bag
        self.assertTrue(set_bagging_state_srv("everything", True).success)
        self.assertTrue(set_bagging_state_srv("nav", True).success)
        
        # Wait a bit to make sure the status is updated
        time.sleep(0.5)
        self.assertEquals(bagging_state_cb.bag_names_states["everything"], True)
        self.assertEquals(bagging_state_cb.bag_names_states["nav"], True)
        self.assertEquals(bagging_state_cb.received_msgs, 3)

        # Turn on the everything and nav bags again - they're already on so they should just stay that way
        self.assertFalse(set_bagging_state_srv("everything", True).success)
        self.assertFalse(set_bagging_state_srv("nav", True).success)

        # Wait a bit to make sure the status is updated
        time.sleep(1.0)
        self.assertEquals(bagging_state_cb.bag_names_states["everything"], True)
        self.assertEquals(bagging_state_cb.bag_names_states["nav"], True)

        # Set bag states to false - reset state
        self.assertTrue(set_bagging_state_srv("everything", False))
        self.assertTrue(set_bagging_state_srv("nav", False))
        
    def test_absent_set_bag_state(self):
        """Test attempting to turn on a non-existant bag profile"""

        set_bagging_state_srv = rospy.ServiceProxy('/bagger/set_bag_state', SetBagState)
        
        # Turn on a non-existant bag
        set_bagging_state_srv("incorrect", True)
        self.assertFalse(set_bagging_state_srv("missing", True).success)

        # Set bag states to false - reset state
        self.assertTrue(set_bagging_state_srv("everything", False))
        self.assertTrue(set_bagging_state_srv("nav", False))
        
    @unittest.skip("Something is wrong with spawning the rosbag processes in Docker") 
    def test_bag_creation_and_bagging(self):
        """Turn on bag profiles, publish a bunch of applicable ros messages, then make sure the bags captured them"""

        # Make sure both bag profiles are recording
        set_bagging_state_srv = rospy.ServiceProxy('/bagger/set_bag_state', SetBagState)
        set_bagging_state_srv("everything", True)
        set_bagging_state_srv("nav", True)
        
        rospy.sleep(1.0)
        
        # Set up publisher for both odom and point messages
        odom_pub = rospy.Publisher('odometry', Odometry, queue_size=10)
        point_pub = rospy.Publisher('point', Point, queue_size=10)
        
        for i in range(0,1000):
            # Publish a bunch of point and odometry messages
            odom = Odometry()
            odom.header.frame_id = "body"
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = random() * 100
            odom.pose.pose.position.y = random() * 100
            odom.pose.pose.position.z = random() * 100
            odom.pose.pose.orientation.x = random()
            odom.pose.pose.orientation.y = random()
            odom.pose.pose.orientation.z = random()
            odom.pose.pose.orientation.w = random()
            odom.twist.twist.linear.x = random()
            odom.twist.twist.linear.y = random()
            odom.twist.twist.linear.z = random()
            odom.twist.twist.angular.x = random()
            odom.twist.twist.angular.y = random()
            odom.twist.twist.angular.z = random()
            
            point = Point()
            point.x = random() * 10
            point.y = random() * 10
            point.z = random() * 10
            
            odom_pub.publish(odom)
            point_pub.publish(point)
            rospy.sleep(0.005)
        
        # Sleep for a little bit, the turn both bag profiles off
        rospy.sleep(1.0)
        
        self.assertTrue(set_bagging_state_srv("everything", False))
        self.assertTrue(set_bagging_state_srv("nav", False))
        
        rospy.sleep(1.0)
        
        # verify that bags have been created in the appropriate directory and that they contain the correct messages
        everything_bag_info = subprocess.check_output(["rosbag", "info", "everything.bag"])
        self.assertTrue("nav_msgs/Odometry" in everything_bag_info)
        self.assertTrue("geometry_msgs/Point" in everything_bag_info)
        
        nav_bag_info = subprocess.check_output(["rosbag", "info", "nav.bag"])
        self.assertTrue("nav_msgs/Odometry" in nav_bag_info)
        self.assertFalse("geometry_msgs/Point" in nav_bag_info)

        # Set bag states to false - reset state
        self.assertTrue(set_bagging_state_srv("everything", False))
        self.assertTrue(set_bagging_state_srv("nav", False))
        

if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestBaggerNode)
