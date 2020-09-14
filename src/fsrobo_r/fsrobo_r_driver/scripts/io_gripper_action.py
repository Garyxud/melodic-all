#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandResult
from fsrobo_r_driver.io_interface import IOInterface


class IOGripperActionServer:
  def __init__(self, action_name):
    rospy.init_node(action_name)
    rospy.loginfo('gripper_action server started')

    self._tool_io_param = rospy.get_param('tool_io')
    robot_ns = self._tool_io_param.get('robot_ns')
  
    rospy.loginfo('wating I/O interface')
    self._io = IOInterface(robot_ns, init_node=False)
    rospy.loginfo('I/O found')
    self.action_server = actionlib.SimpleActionServer(action_name, GripperCommandAction, self.execute_action, False)
  
  def run(self):
    self.action_server.start()
    rospy.loginfo('action server started')
    rospy.spin()

  def _open(self):
    return self._io_operation('open')
  
  def _close(self):
    return self._io_operation('close')

  def _io_operation(self, name):
    param = self._tool_io_param.get(name)
    print('_io_operation: {} {}'.format(name, param))

    for x in param['trigger']:
      addr, data = self._translate_io_param(x)
      self._io.set_digital_val(addr, *data)
    for x in param['state']:
      addr, data = self._translate_io_param(x)
      print('wait state: {} {}'.format(addr, data))
      result = self._io.wait_digital_val(addr, data, time_out=1.0)
      print('result: {}'.format(result))
      if not result:
        return False
    return True

  def _translate_io_param(self, param):
    start_addr = min(map(int, param.keys()))
    end_addr = max(map(int, param.keys()))

    data = [param.get(str(k), -1) for k in range(start_addr, end_addr + 1)]

    return start_addr, data

  def execute_action(self, goal):
    print('------------------execute_action')
    print(goal)
    if goal.command.position > 0:
      success = self._open()
    else:
      success = self._close()

    if success:
      result = GripperCommandResult()
      result.position = goal.command.position
      result.effort = goal.command.max_effort
      result.reached_goal = True
      print(result)
      self.action_server.set_succeeded(result)
      print('------------------set_succeeded')
    else:
      rospy.loginfo('tool operation aboarted')
      self.action_server.set_aborted()
      print('------------------set_aborted')

if __name__ == '__main__':
  server = IOGripperActionServer('gripper_action')
  server.run()