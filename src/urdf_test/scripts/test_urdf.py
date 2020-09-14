#!/usr/bin/env python

import rospy
import unittest

class TestURDF(unittest.TestCase):

    def test_urdf(self):
        self.assertTrue(rospy.has_param("robot_description"))
        rd = rospy.get_param("robot_description")
        self.assertNotEqual("", rd)

if __name__ == '__main__':
    import rosunit
    rospy.init_node("test_urdf")
    rosunit.unitrun("urdf_test", 'urdf_test', TestURDF)
