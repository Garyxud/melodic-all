#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# About
##############################################################################
#
# Simple script to publish markers for the tutorial.
#
##############################################################################
# Imports
##############################################################################

import rospy
import visualization_msgs.msg as visualization_msgs


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    transitions = {visualization_msgs.Marker.CUBE: visualization_msgs.Marker.SPHERE,
                   visualization_msgs.Marker.SPHERE: visualization_msgs.Marker.ARROW,
                   visualization_msgs.Marker.ARROW: visualization_msgs.Marker.CYLINDER,
                   visualization_msgs.Marker.CYLINDER: visualization_msgs.Marker.CUBE,
                   }
    rospy.init_node('publish_markers')
    shape = visualization_msgs.Marker.CUBE
    publisher = rospy.Publisher('shapes', visualization_msgs.Marker, latch=True, queue_size=5)
    while not rospy.is_shutdown():
        marker = visualization_msgs.Marker()
        marker.header.frame_id = "my_frame"
        marker.header.stamp = rospy.rostime.Time.now()
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = shape
        marker.action = visualization_msgs.Marker.ADD
        # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.rostime.Duration()

        publisher.publish(marker)

        # Cycle between different shapes
        # print("Transitions: %s" % transitions)
        shape = transitions[shape]
        # print("Shape: %s" % shape)
        rospy.rostime.wallsleep(1.0)
