#!/usr/bin/env python
from __future__ import absolute_import

import functools

"""
 A very simple echo ROS node class.
 - echo from topic to echo_topic
 - echo service
"""

import roslib
import rospy
import std_msgs.msg as std_msgs


# TODO : get rid of this somehow ( dynamic generation or integration of more basic services in ROS )
from pyros_test.srv import StringEchoService


class EchoNodeArgumentNotFound(Exception):
    pass


class EchoNode(object):
    def __init__(self):
        rospy.loginfo('String Echo node started. [' + rospy.get_name() + ']')

        topic_name = rospy.get_param("~topic_name", "topic")
        print 'Parameter {0!s} has value {1!s}'.format(rospy.resolve_name('~topic_name'), topic_name)
        if topic_name == "":
            raise EchoNodeArgumentNotFound("{0} parameter not found".format(rospy.resolve_name('~topic_name')))

        echo_topic_name = rospy.get_param("~echo_topic_name", "echo_topic")
        print 'Parameter {0!s} has value {1!s}'.format(rospy.resolve_name('~echo_topic_name'), echo_topic_name)
        if echo_topic_name == "":
            raise EchoNodeArgumentNotFound("{0} parameter not found".format(rospy.resolve_name('~echo_topic_name')))

        echo_service_name = rospy.get_param("~echo_service_name", "echo_service")
        print 'Parameter {0!s} has value {1!s}'.format(rospy.resolve_name('~echo_service_name'), echo_service_name)
        if echo_service_name == "":
            raise EchoNodeArgumentNotFound("{0} parameter not found".format(rospy.resolve_name('~echo_service_name')))

        # TODO parameter topic type to reuse this for *any* msg type

        pub = rospy.Publisher(echo_topic_name, std_msgs.String, queue_size=1)

        # building callback
        echo = functools.partial(self.topic_callback, data_type=std_msgs.String, pub=pub)
        sub = rospy.Subscriber(topic_name, std_msgs.String, echo)

        srv = rospy.Service(echo_service_name, StringEchoService, self.service_callback)


    # a callback that just echoes the message ( if it doesnt come from me )
    def topic_callback(self, data, data_type, pub):
        # extract data
        print "==> echoing {d} ".format(d=data)
        pub.publish(data=data.data)  # data member is needed because it s the only std_msgs.String.__slots__
        # BAD ROS API is not symmetrical
        # TODO : generic way to forward any msgtype safely
        pass


    def service_callback(self, data):
        # extract data
        print "==> echoing {d} ".format(d=data)
        return data.request  # why specific field here as well ? "request" / response but no "data" ?
        # TODO : generic way to forward any msgtype safely


    def spin(self):
        rospy.spin()
