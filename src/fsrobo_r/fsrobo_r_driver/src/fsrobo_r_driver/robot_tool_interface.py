#!/usr/bin/env python

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
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommand
import actionlib
import moveit_commander
from fsrobo_r_driver.geometry_util import GeometryUtil
import tf
import math

class RobotToolInterface:
    def __init__(self, tool_name=None):
        if tool_name:
            self.ns = '/' + tool_name + '/'
        else:
            self.ns = '/'
        self._offset = rospy.get_param(self.ns + 'tool/offset', [0, 0, 0, 0, 0, 0])
        self._action_ns = rospy.get_param(self.ns + 'tool/action_ns', None)
        print(self._action_ns)
        self._origin_link = rospy.get_param(self.ns + 'tool/origin_link', None)
        self._planning_group = rospy.get_param(self.ns + 'tool/planning_group', None)

        self._geometry_util = GeometryUtil()

        if self._action_ns:
            self._open_param = rospy.get_param(self.ns + 'tool/open_param')
            self._close_param = rospy.get_param(self.ns + 'tool/close_param')
            self._client = actionlib.SimpleActionClient(self._action_ns, GripperCommandAction)
        else:
            self._open_param = None
            self._close_param = None
            self._client = None

        if self._origin_link and self._planning_group:
            self._move_group = moveit_commander.MoveGroupCommander(self._planning_group)
            ee_link = self._move_group.get_end_effector_link()
            pose = self._geometry_util.get_current_pose(self._origin_link, ee_link)
            tool_pos = map(lambda x: x * 1000.0, pose[0])
            tool_ori = map(lambda x: math.degrees(x),tf.transformations.euler_from_quaternion(pose[1], axes='rzyx'))
            self._offset = tool_pos + tool_ori
        else:
            self._offset = [0, 0, 0, 0, 0, 0]
    
    def open(self, wait=True):
        if self._client and self._client.wait_for_server(rospy.Duration(5)):
            goal = GripperCommandGoal()
            goal.command.position = self._open_param[0]
            goal.command.max_effort = self._open_param[1]
            self._client.send_goal(goal)
            if wait:
                return self._client.wait_for_result(rospy.Duration(5))
            else:
                rospy.sleep(3)
                return True
        else:
            return False

    def close(self, wait=True):
        if self._client and self._client.wait_for_server(rospy.Duration(5)):
            goal = GripperCommandGoal()
            goal.command.position = self._close_param[0]
            goal.command.max_effort = self._close_param[1]
            self._client.send_goal(goal)
            if wait:
                return self._client.wait_for_result(rospy.Duration(5))
            else:
                rospy.sleep(3)
                return True
        else:
            return False

    def get_offset(self, index=0):
        return self._offset

if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    #tool = RobotToolInterface('arm2/tool2')
    #tool = RobotToolInterface('generic_hand')
    tool = RobotToolInterface('tool1')
    print(tool.get_offset())
    #print tool.open(wait=False)
    #print tool.close(wait=False)