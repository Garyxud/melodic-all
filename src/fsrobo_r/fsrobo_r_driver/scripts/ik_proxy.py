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

from moveit_msgs.srv import GetPositionIK, GetPositionIKResponse
import rospy
from fsrobo_r_driver.robot_interface import RobotInterface
from fsrobo_r_driver.robot_tool_interface import RobotToolInterface

class IKProxyService:
  def __init__(self):
    self._last_offset = []

  def load_config(self):
    self._config = {}

    params = rospy.get_param('ik_proxy')

    robot_interface = {}
    tool_interface = {'': None}
    joint_names = {}

    for x in params:
      group_name = x['group_name']
      robot_name = x['robot_name']
      tool_name = x['tool_name']

      if not robot_name in robot_interface:
        robot_interface[robot_name] = RobotInterface(robot_name)
        if robot_name:
          ns = robot_name + '/'
        else:
          ns = ''
        joint_names[robot_name] = rospy.get_param(ns + 'controller_joint_names')

      if not tool_name in tool_interface:
        tool_interface[tool_name] = RobotToolInterface(tool_name)

      self._config[group_name] = {
        'robot': robot_interface[robot_name],
        'tool': tool_interface[tool_name],
        'joint': joint_names[robot_name] }

  def handle_solve_ik(self, req):
    print "-----------request!"
    pos = req.ik_request.pose_stamped.pose.position
    ori = req.ik_request.pose_stamped.pose.orientation
    group_name = req.ik_request.group_name

    tool = self._config[group_name]['tool']
  
    res = GetPositionIKResponse()
  
    print pos
    print ori
    rb = self._config[group_name]['robot']
    if tool:
      print('tool_offset: {}'.format(tool.get_offset()))
      offset = tool.get_offset()
      if self._last_offset != offset:
        rb.set_tool(tool)
        self._last_offset = offset
    else:
      if self._last_offset != []:
        rb.set_tool(tool)
        self._last_offset = []

    joint = rb.get_position_ik([pos.x, pos.y, pos.z], [ori.x, ori.y, ori.z, ori.w])
    res.solution.joint_state.header = req.ik_request.pose_stamped.header
    if len(joint) != 0:
      print "pose!"
      print req.ik_request.pose_stamped.pose
      print "req!"
      print req.ik_request
      print "joint!"
      print joint
      print "success!"
      #res.solution.joint_state.name = self._config[group_name]['joint'] + ['left_ezgripper_knuckle_palm_L1_1', 'left_ezgripper_knuckle_L1_L2_1', 'left_ezgripper_knuckle_palm_L1_2', 'left_ezgripper_knuckle_L1_L2_2']
      res.solution.joint_state.name = self._config[group_name]['joint']
      #res.solution.joint_state.position = joint + [0, 0, 0, 0]
      res.solution.joint_state.position = joint
      res.error_code.val = res.error_code.SUCCESS
      #res.error_code.val = 10
      print '>--------------res'
      print res
      print '<--------------res'
      print res.solution.joint_state.position
    else:
      print('NO IK!')
      res.error_code.val = res.error_code.NO_IK_SOLUTION
  
    return res

  def run(self):
    rospy.init_node('ik_proxy')
    self.load_config()
    s = rospy.Service('solve_ik', GetPositionIK, self.handle_solve_ik)
    print "Ready to solve ik."
    rospy.spin()
  
if __name__ == "__main__":
    s = IKProxyService()
    s.run()