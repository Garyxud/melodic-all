#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import WrenchStamped, Wrench
from fsrobo_r_msgs.srv import GetWrench

class ForcePublisher(object):
  def __init__(self):
    self._zero_offset = Wrench()
    self._reset_time = rospy.Time.now()
    self._zero_offset_buffer = []
    self._resetting = True
    self.wrench = Wrench()

  def cb_raw_force(self, msg):
    if self._reset_time > rospy.Time.now():
      self._zero_offset_buffer.append(msg)

    else:
      if self._resetting:
        rospy.logwarn("reset!")
        self._zero_offset = self._avg_wrench(self._zero_offset_buffer)
        self._resetting = False
      ws = WrenchStamped()
      ws.header = msg.header
      ws.wrench.force.x = msg.wrench.force.x - self._zero_offset.force.x
      ws.wrench.force.y = msg.wrench.force.y - self._zero_offset.force.y
      ws.wrench.force.z = msg.wrench.force.z - self._zero_offset.force.z
      ws.wrench.torque.x = msg.wrench.torque.x - self._zero_offset.torque.x
      ws.wrench.torque.y = msg.wrench.torque.y - self._zero_offset.torque.y
      ws.wrench.torque.z = msg.wrench.torque.z - self._zero_offset.torque.z
      self.wrench = ws.wrench
      self._pub.publish(ws)

  def _avg_wrench(self, wrench_list):
    w = Wrench()
    for x in wrench_list:
      w.force.x += x.wrench.force.x
      w.force.y += x.wrench.force.y
      w.force.z += x.wrench.force.z
      w.torque.x += x.wrench.torque.x
      w.torque.y += x.wrench.torque.y
      w.torque.z += x.wrench.torque.z

    n = len(wrench_list)
    if n:
      w.force.x /= n
      w.force.y /= n
      w.force.z /= n
      w.torque.x /= n
      w.torque.y /= n
      w.torque.z /= n

    return w

  def cb_reset(self, req):
    self._zero_offset_buffer = []
    self._reset_time = rospy.Time.now() + rospy.Duration(1)
    self._resetting = True
    rospy.logwarn("cb_reset!")

    return EmptyResponse()
    
  def cb_get_current_force(self, req):
    return self.wrench

  def run(self):
    self._pub = rospy.Publisher('force', WrenchStamped, queue_size=10)
    rospy.Subscriber('raw_force', WrenchStamped, self.cb_raw_force)
    rospy.Service('~reset', Empty, self.cb_reset)
    rospy.Service('get_current_force', GetWrench, self.cb_get_current_force)
    rospy.spin()

if __name__ == '__main__':
  rospy.init_node('force_publisher')
  fp = ForcePublisher()
  fp.run()