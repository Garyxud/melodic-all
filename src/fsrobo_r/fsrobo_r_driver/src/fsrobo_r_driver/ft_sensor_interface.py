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
from fsrobo_r_msgs.srv import SetStopForce, GetWrench
from std_srvs.srv import Empty, Trigger

class FTSensorInterface:
  
    def __init__(self, robot_name=None):
        if robot_name == '':
            self.ns = '/'
        elif robot_name:
            self.ns = '/' + robot_name + '/'
        else:
            self.ns = ''
        self._set_stop_force = rospy.ServiceProxy(self.ns + 'set_stop_force', SetStopForce)
        self._clear_stop_force = rospy.ServiceProxy(self.ns + 'clear_stop_force', Empty)
        self._get_current_force = rospy.ServiceProxy(self.ns + 'get_current_force', GetWrench)
        self._is_stopped = rospy.ServiceProxy(self.ns + 'is_stopped', Trigger)

    def set_stop_force(self, axis, val):
        rospy.wait_for_service(self.ns + 'set_stop_force', timeout=3)
        self._set_stop_force(axis, val)

    def clear_stop_force(self):
        rospy.wait_for_service(self.ns + 'clear_stop_force', timeout=3)
        self._clear_stop_force()

    def get_current_force(self):
        rospy.wait_for_service(self.ns + 'get_current_force', timeout=3)
        return self._get_current_force()

    def is_stopped(self):
        rospy.wait_for_service(self.ns + 'is_stopped', timeout=3)
        res = self._is_stopped()
        return res.success
   
if __name__ == '__main__':
    ft = FTSensorInterface()
    ft.clear_stop_force()
    ft.set_stop_force('z', -3)
    print(ft.is_stopped())
