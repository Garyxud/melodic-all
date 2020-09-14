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

import sys
import rospy
from fsrobo_r_msgs.srv import *
from fsrobo_r_msgs.msg import *
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from fsrobo_r_driver.robot_program_interface import RobotProgramInterface
import json

class RobotInterface:
    ROBOT_PROGRAM_DIR = '/opt/ros/scripts'

    def __init__(self, robot_name=None):
        if robot_name:
            self.ns = '/' + robot_name + '/'
        else:
            self.ns = '/'
        self._get_position_ik = rospy.ServiceProxy(self.ns + 'solve_ik', GetPositionIK)
        self._set_posture = rospy.ServiceProxy(self.ns + 'set_posture', SetPosture)
        self._get_posture = rospy.ServiceProxy(self.ns + 'get_posture', GetPosture)
        self._set_tool_offset = rospy.ServiceProxy(self.ns + 'set_tool_offset', SetToolOffset)

        self._tool = None
        self._joint_speed = 25
        self._line_speed = 25

        self._rpi = RobotProgramInterface(robot_name)
    
    def get_position_ik(self, position, orientation):
        rospy.wait_for_service(self.ns + 'solve_ik')

        req = GetPositionIKRequest()
        req.ik_request.pose_stamped.pose.position.x = position[0]
        req.ik_request.pose_stamped.pose.position.y = position[1]
        req.ik_request.pose_stamped.pose.position.z = position[2]
        req.ik_request.pose_stamped.pose.orientation.x = orientation[0]
        req.ik_request.pose_stamped.pose.orientation.y = orientation[1]
        req.ik_request.pose_stamped.pose.orientation.z = orientation[2]
        req.ik_request.pose_stamped.pose.orientation.w = orientation[3]
        result = self._get_position_ik(req)

        if result.error_code.val == result.error_code.SUCCESS:
          return list(result.solution.joint_state.position)
        else:
          return []

    def set_posture(self, posture):
        rospy.wait_for_service(self.ns + 'set_posture')
        self._set_posture(posture)
    
    def get_posture(self):
        rospy.wait_for_service(self.ns + 'get_posture')
        result = self._get_posture()
        return result.posture

    def set_tool(self, tool):
        print('set_tool!')
        if tool:
            self._tool = tool
            print(self._tool.get_offset())
            self._set_tool(1, *self._tool.get_offset())
        else:
            self._tool = None
            self._set_tool(1)

    def _set_tool(self, tool_id, offx=0, offy=0, offz=0, offrz=0, offry=0, offrx=0):
        rospy.wait_for_service(self.ns + 'set_tool_offset')

        req = SetToolOffsetRequest()
        req.origin.x = offx
        req.origin.y = offy
        req.origin.z = offz
        req.rotation.z = offrz
        req.rotation.y = offry
        req.rotation.x = offrx

        return self._set_tool_offset(req)


    #def _set_tool(self, tool_id, offx=0, offy=0, offz=0, offrz=0, offry=0, offrx=0):
    #    param = json.dumps({
    #        'settool': {
    #            'id': tool_id,
    #            'offx': offx,
    #            'offy': offy,
    #            'offz': offz,
    #            'offrz': offrz,
    #            'offry': offry,
    #            'offrx': offrx
    #        },
    #        'changetool': {
    #            'tid': tool_id
    #        }
    #    })

    #    return self._rpi.execute(self.ROBOT_PROGRAM_DIR + '/settool.py', param) 

    def set_joint_speed(self, speed):
        self._joint_speed = speed
    
    def set_line_speed(self, speed):
        self._line_speed = speed

    def line_move(self, x, y, z, rz, ry, rx, posture=7):
        tool_id = 1 if self._tool else 0
        param = json.dumps({
            'changetool': {
                'tid': tool_id
            },
            'motionparam': {
                'lin_speed': self._line_speed
            },
            'line': {
                'x': x,
                'y': y,
                'z': z,
                'rz': rz,
                'ry': ry,
                'rx': rx,
                'posture': posture
            }
        })

        return self._rpi.execute(self.ROBOT_PROGRAM_DIR + '/line.py', param) 

    def tool_move(self, x=0, y=0, z=0, rz=0, ry=0, rx=0):
        tool_id = 1 if self._tool else 0
        param = json.dumps({
            'changetool': {
                'tid': tool_id
            },
            'motionparam': {
                'lin_speed': self._line_speed
            },
            'toolmove': {
                'dx': x,
                'dy': y,
                'dz': z,
                'drz': rz,
                'dry': ry,
                'drx': rx
            }
        })

        return self._rpi.execute(self.ROBOT_PROGRAM_DIR + '/toolmove.py', param) 


if __name__ == '__main__':
    rb = RobotInterface()
    print rb.get_position_ik([0.4, 0.066, 0.663], [0, 0, 0, 1])
    rb.set_posture(5)
    print rb.get_posture()
    rb.set_posture(3)
    print rb.get_posture()
 