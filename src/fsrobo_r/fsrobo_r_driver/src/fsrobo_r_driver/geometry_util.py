#!/usr/bin/env python
# -*- coding: utf-8 -*-

# FSRobo-R Package BSDL
# ---------
# Copyright (C) 2019 FUJISOFT. All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# --------

import rospy
import time
import tf
import fnmatch
from geometry_msgs.msg import PoseStamped

class GeometryUtil:
  def __init__(self, init_node=False):
    if init_node:
      rospy.init_node('tf_listener', anonymous=True)
    self._listener = tf.TransformListener()
    rospy.sleep(0.5)

  def transform_pose(self, target_frame, source_frame, position, orientation):
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now() - rospy.Duration(0.1)
    pose_stamped.header.frame_id = source_frame
    pose_stamped.pose.position.x = position[0]
    pose_stamped.pose.position.y = position[1]
    pose_stamped.pose.position.z = position[2]
    pose_stamped.pose.orientation.x = orientation[0]
    pose_stamped.pose.orientation.y = orientation[1]
    pose_stamped.pose.orientation.z = orientation[2]
    pose_stamped.pose.orientation.w = orientation[3]

    self._listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4))

    trans_result = self._listener.transformPose(target_frame, pose_stamped)

    return ([trans_result.pose.position.x,
             trans_result.pose.position.y,
             trans_result.pose.position.z],
            [trans_result.pose.orientation.x,
             trans_result.pose.orientation.y,
             trans_result.pose.orientation.z,
             trans_result.pose.orientation.w])
  
  def get_current_pose(self, target_frame, source_frame, timeout=4.0):
    t = time.time()
    while time.time() - t < timeout:
      try:
        now = rospy.Time.now()
        self._listener.waitForTransform(target_frame, source_frame, now, rospy.Duration(0.01))
        trans_result = self._listener.lookupTransform(target_frame, source_frame, now)
        return trans_result
      except tf.Exception as e:
        pass

    return None

  def get_frame_names(self, pattern=None):
    names = self._listener.getFrameStrings()
    if pattern is None:
      return names
    else:
      return fnmatch.filter(names, pattern)

#if __name__ == '__main__':
#  rospy.init_node('tf_test', anonymous=True)
#  rospy.loginfo('node started')
#  util = GeometryUtil()
#  print  util.transform_pose('arm2/base_link', 'world', \
#    [0.1003, 0.0, 0.8325], [0.0, 0.0, -0.707, 0.707])
#  print  util.transform_pose('world', 'arm2/base_link', \
#    [0.1003, 0.0, 0.8325], [0.0, 0.0, -0.707, 0.707])
#  print  util.transform_pose('arm1/base_link', 'arm2/base_link', \
#    [0.1003, 0.0, 0.8325], [0.0, 0.0, -0.707, 0.707])
#  print util.get_current_pose('arm2/base_link', 'arm2/Link6')
#  print util.get_current_pose('arm1/base_link', 'arm1/Link6')
#  print util.get_current_pose('arm1/Link6', 'arm1/base_link')
#  print util.get_current_pose('world', 'arm2/Link6')
#
#  #listener = tf.TransformListener()
#  #listener.waitForTransform('world', 'arm2/Link6', rospy.Time(0), rospy.Duration(4))
#  #listener.waitForTransform('arm2/base_link', 'arm2/Link6', rospy.Time(0), rospy.Duration(4))
#  #print listener.allFramesAsString()
#  #print listener.transformPose('world', pose)
#  #print listener.lookupTransform('arm2/base_link', 'arm2/Link6', rospy.Time(0))

if __name__ == '__main__':
  rospy.init_node('tf_test', anonymous=True)
  rospy.loginfo('node started')
  util = GeometryUtil()
  #ret = util.get_current_pose('camera_color_optical_frame', 'object_22_0', timeout=0.5)
  #print(ret)
  while not rospy.is_shutdown():
    ret = util.get_current_pose('camera_color_optical_frame', 'object_22_0', timeout=0.1)
    print(ret)
