#!/usr/bin/env python

""" Testing the find_node function """

import unittest
import rospy
import rostest
import rocon_python_comms


class TestFindNode(unittest.TestCase):
    def test_find_node_unique(self):
        node_name = rospy.get_param('unique_node_name', 'unique_node')
        try:
            found_nodes = rocon_python_comms.find_node(node_name, unique=True)
            rospy.loginfo('Found node(s): ' + str(found_nodes))
        except rocon_python_comms.NotFoundException as e:
            rospy.logerr("Node: '" + node_name + "' not found or too many found: " + str(e))
            assert True

    def test_find_node_non_unique(self):
        node_name = rospy.get_param('non_unique_node_name', 'non_unique_node')
        try:
            found_nodes = rocon_python_comms.find_node(node_name, unique=False)
            rospy.loginfo('Found node(s): ' + str(found_nodes))
            if len(found_nodes) < 2:
                rospy.loginfo('Not enough nodes found (min = 2): ' + str(found_nodes))
                assert True
        except rocon_python_comms.NotFoundException as e:
            rospy.logerr("Node: '" + node_name + "' not found: " + str(e))
            assert True

    def test_not_find_node(self):
        node_name = rospy.get_param('non_existent_node_name', 'non_existent_node')
        try:
            found_nodes = rocon_python_comms.find_node(node_name, unique=False)
            rospy.logerr("Found non-existent node '" + node_name + "'. Not good!")
            assert True
        except rocon_python_comms.NotFoundException as e:
            rospy.loginfo("Non-existent node not found. Good!")

if __name__ == '__main__':
    rostest.rosrun('rocon_python_comms',
                   'test_find_node',
                   TestFindNode)
