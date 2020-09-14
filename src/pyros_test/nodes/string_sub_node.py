#!/usr/bin/env python

""" A dummy ROS node """
from __future__ import absolute_import

import common
import sys

import rospy
import std_msgs

# TODO : implement service to check if sub


# a callback that just echoes the message ( if it doesnt come from me )
def topic_callback(self, data):
    pass


if __name__ == '__main__':
    try:
        args = rospy.myargv(argv=sys.argv)
        node_name = args[1] if len(args) > 1 else 'string_pub_node'

        rospy.init_node(node_name)
        rospy.loginfo('String Pub node started. [' + rospy.get_name() + ']')

        topic_name = rospy.get_param("~topic_name", "")
        print 'Parameter {0!s} has value {1!s}'.format(rospy.resolve_name('~topic_name'), topic_name)
        if topic_name == "":
            print "{0} parameter not found".format(rospy.resolve_name('~topic_name'))
            raise common.TestArgumentNotFound
        test_message = rospy.get_param("~test_message", "")
        print 'Parameter {0!s} has value {1!s}'.format(rospy.resolve_name('~test_message'), test_message)
        if test_message == "":
            print "{0} parameter not found".format(rospy.resolve_name('~test_message'))
            raise common.TestArgumentNotFound

        sub = rospy.Subscriber(topic_name, std_msgs.msg.String, topic_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
