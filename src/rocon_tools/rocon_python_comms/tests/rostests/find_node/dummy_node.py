#!/usr/bin/env python

""" A dummy ROS node """

import rospy


if __name__ == '__main__':
    try:
        rospy.init_node('dummy_node')
        rospy.loginfo('Dummy node started. [' + rospy.get_name() + ']')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
