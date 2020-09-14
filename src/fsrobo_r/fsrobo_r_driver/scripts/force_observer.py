#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import threading
from geometry_msgs.msg import Wrench, WrenchStamped
from fsrobo_r_msgs.srv import SetStopForce, SetStopForceResponse
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from std_msgs.msg import String

class ForceObserver(object):
  def __init__(self, group_name):
    #self._group = moveit_commander.MoveGroupCommander(group_name, ns="/")
    self._handlers = {}
    self._current_wrench = Wrench()
    self._lock = threading.Lock()
    self._is_stopped = False

  def _cb_set_stop_force(self, req):

    if req.axis in (req.AXIS_X, req.AXIS_Y, req.AXIS_Z):
      threshold = getattr(self._current_wrench.force, req.axis) + req.val
      info = 'axis: {}, val: {}, threshold: {}'.format(req.axis, req.val, threshold)
      rospy.loginfo('set_stop_force: {}'.format(info))
      if req.val >= 0:
        self.add_handler(lambda msg: getattr(msg.wrench.force, req.axis) > threshold, lambda msg: self.stop(), info)
      else:
        self.add_handler(lambda msg: getattr(msg.wrench.force, req.axis) < threshold, lambda msg: self.stop(), info)
    elif req.axis in (req.AXIS_MX, req.AXIS_MY, req.AXIS_MZ):
      threshold = getattr(self._current_wrench.torque, req.axis) + req.val
      info = 'axis: {}, val: {}, threshold: {}'.format(req.axis, req.val, threshold)
      rospy.loginfo('set_stop_force: {}'.format(info))
      if req.val >= 0:
        self.add_handler(lambda msg: getattr(msg.wrench.torque, req.axis) > threshold, lambda msg: self.stop(), info)
      else:
        self.add_handler(lambda msg: getattr(msg.wrench.torque, req.axis) < threshold, lambda msg: self.stop(), info)
    else:
      raise rospy.ServiceException('invalid axis: {}'.format(req.axis))

    res = SetStopForceResponse()
    return res

  def _cb_clear_stop_force(self, req):
    rospy.loginfo('clear_stop_force')
    with self._lock:
      self._handlers = {}
      self._is_stopped = False
    res = EmptyResponse()
    return res

  def _cb_is_stopped(self, req):
    res = TriggerResponse()
    res.success = self._is_stopped
    return res

  def stop(self):
    #self._group.stop()
    self._is_stopped = True
    self._pub.publish('stop')

  def add_handler(self, condition, command, info='', key='default'):
    with self._lock:
      self._handlers['default'] = {'condition': condition, 'command': command, 'info': info}

  def _cb_wrench_stamped(self, msg):
    self._current_wrench = msg.wrench
    new_handlers = {}
    with self._lock:
      for k, x in self._handlers.iteritems():
        if x['condition'](msg):
          rospy.loginfo('command is called!: {}'.format(x['info']))
          x['command'](msg)
        else:
          new_handlers[k] = x

      self._handlers = new_handlers

  def run(self):
    rospy.Subscriber('force', WrenchStamped, self._cb_wrench_stamped)
    rospy.Service('set_stop_force', SetStopForce, self._cb_set_stop_force)
    rospy.Service('clear_stop_force', Empty, self._cb_clear_stop_force)
    rospy.Service('is_stopped', Trigger, self._cb_is_stopped)
    self._pub = rospy.Publisher('/trajectory_execution_event', String, queue_size=10)

def main():
  rospy.init_node('force_observer', anonymous=True)
  fo = ForceObserver(sys.argv[1])
  fo.run()
  rospy.spin()

if __name__ == '__main__':
  main()