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

import math
import rospy
import tf
import moveit_commander
from moveit_msgs.srv import GetPositionIK, GetPositionIKResponse
from fsrobo_r_msgs.srv import SetToolOffset, SetToolOffsetRequest
from fsrobo_r_driver.geometry_util import GeometryUtil
from fsrobo_r_driver.cc_client import CCClient


class RobotControllerInterface(object):
    def __init__(self, ip):
        self._api = CCClient(ip_address=ip)
        self._set_tool_offset = rospy.ServiceProxy('set_tool_offset', SetToolOffset)

    def set_tool_offset(self, offx=0, offy=0, offz=0, offrz=0, offry=0, offrx=0):
        rospy.wait_for_service('set_tool_offset')

        req = SetToolOffsetRequest()
        req.origin.x = offx
        req.origin.y = offy
        req.origin.z = offz
        req.rotation.z = offrz
        req.rotation.y = offry
        req.rotation.x = offrx

        return self._set_tool_offset(req)

    def solve_ik(self, position, orientation):
        data = {}
        angle = list(self.quaternion_to_euler(orientation))

        pos = map(lambda x: x * 1000, [position.x, position.y, position.z])
        deg = map(lambda x: math.degrees(x), angle)

        if self._api.position_to_joint(pos[0], pos[1], pos[2], deg[2], deg[1], deg[0], data) == 0:
            result = map(lambda k: data['DA'][k], ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
            # TODO IKからジョイントの制限を超える角度が返ってくる場合がある
            for i in range(5):
                if result[i] > 240:
                    result[i] -= 360
                elif result[i] < -240:
                    result[i] += 360
            if result[5] > 360:
                result[i] -= 360
            elif result[5] < -360:
                result[5] += 360
            result = map(lambda x: math.radians(x), result)
        else:
            result = []

        return result
  
    def quaternion_to_euler(self, quaternion):
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w), 'rzyx')
        return e


class RobootToolUtil(object):
    def __init__(self):
        self._gu = GeometryUtil()
        self._rc = None
        self._tool_base_link = rospy.get_namespace() + 'Link6'

    @property
    def rc(self):
        if self._rc is None:
            self._rc = moveit_commander.RobotCommander()
            return self._rc
        else:
            return self._rc

    def get_tool_offset(self, group_name):
        group = self.rc.get_group(group_name)

        ee_link = group.get_end_effector_link()
        pose = self._gu.get_current_pose(self._tool_base_link, ee_link)
        tool_pos = map(lambda x: x * 1000.0, pose[0])
        tool_ori = map(lambda x: math.degrees(x), tf.transformations.euler_from_quaternion(pose[1], axes='rzyx'))

        tool_offset = tool_pos + tool_ori

        return tool_offset


class IKService:
    def __init__(self):
        ip = rospy.get_param('robot_ip_address', '192.168.0.23')
        self._rb = RobotControllerInterface(ip)
        self._tool_util = RobootToolUtil()

        default_joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
        self.joint_names = rospy.get_param('controller_joint_names', default_joint_names)

        self._tool_offset_cache = {}
        self._last_tool_offset = []

    def update_tool_offset(self, group_name):
        if group_name in self._tool_offset_cache:
            tool_offset = self._tool_offset_cache[group_name]
        else:
            tool_offset = self._tool_util.get_tool_offset(group_name)
            self._tool_offset_cache[group_name] = tool_offset

        if tool_offset != self._last_tool_offset:
            self._rb.set_tool_offset(*tool_offset)
            self._last_tool_offset = tool_offset

    def handle_solve_ik(self, req):
        #print("-----------request!")
        pos = req.ik_request.pose_stamped.pose.position
        ori = req.ik_request.pose_stamped.pose.orientation
        group_name = req.ik_request.group_name

        #print(pos)
        #print(ori)
        #print('group_name: {}'.format(group_name))

        res = GetPositionIKResponse()
        res.solution.joint_state.header = req.ik_request.pose_stamped.header

        self.update_tool_offset(group_name)
        joint = self._rb.solve_ik(pos, ori)

        if len(joint) != 0:
            #print("pose!")
            #print(req.ik_request.pose_stamped.pose)
            #print("req!")
            #print(req.ik_request)
            #print("joint!")
            #print(joint)
            #print("success!")
            res.solution.joint_state.name = self.joint_names
            res.solution.joint_state.position = joint
            res.error_code.val = res.error_code.SUCCESS
            print(res.solution.joint_state.position)
        else:
            res.error_code.val = res.error_code.NO_IK_SOLUTION

        return res

    def run(self):
        rospy.Service('solve_ik', GetPositionIK, self.handle_solve_ik)
        print("Ready to solve ik.")
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('ik_server')
    s = IKService()
    s.run()
