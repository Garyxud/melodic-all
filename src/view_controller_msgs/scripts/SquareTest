#!/usr/bin/env python

import roslib
roslib.load_manifest("view_controller_msgs")

import rospy
from math import *
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import Point, Vector3

rospy.init_node("camera_test", anonymous = True)

pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size = 1)

rate_float = 0.25
rate = rospy.Rate(rate_float)
rospy.sleep(1)

while not rospy.is_shutdown():

  #print "Top of loop!"

  t = rospy.get_time()
  cp = CameraPlacement()
  r = 10

  p = Point(r*cos(2*pi*t/20), r*sin(2*pi*t/20), 2)
  #p = Point(5,5,0)
  cp.eye.point = p
  cp.eye.header.frame_id = "base_link"

  #f = Point(0, 0, 2*cos(2*pi*t/5))
  f = Point(0, 0, 0)
  cp.focus.point = f
  cp.focus.header.frame_id = "base_link"

  up = Vector3(0, 0, 1)
  cp.up.vector = up
  cp.up.header.frame_id = "base_link"

  cp.time_from_start = rospy.Duration(1.0/rate_float)
  print "Publishing a message!"
  pub.publish(cp)
  #print "Sleeping..."
  rate.sleep()
  #print "End of loop!"

