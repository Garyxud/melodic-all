#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from fsrobo_r_driver.robot_interface import RobotInterface
from fsrobo_r_driver.robot_tool_interface import RobotToolInterface

if __name__ == '__main__':
  rospy.init_node('hand_test', anonymous=True)
  robot = RobotInterface()
  tool = RobotToolInterface('generic_hand')
  tool.close()
  tool.open()
  robot.set_tool(tool)
  print robot.tool_move(z=30)