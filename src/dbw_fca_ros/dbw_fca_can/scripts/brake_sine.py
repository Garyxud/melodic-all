#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Dataspeed Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright notice,
#       this list of conditions and the following disclaimer in the documentation
#       and/or other materials provided with the distribution.
#     * Neither the name of Dataspeed Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from this
#       software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import math

from std_msgs.msg import Bool
from dbw_fca_msgs.msg import BrakeCmd


class SineTest:
    def __init__(self):
        rospy.init_node('sine_test')
        self.pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)
        self.sub = rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.recv_enable)

        self.high_peak = rospy.get_param('~high_peak', 0.40)
        self.low_peak = rospy.get_param('~low_peak', 0.15)
        self.period = rospy.get_param('~period', 10)

        self.enabled = False
        self.t = 0
        self.sample_time = 0.02

        rospy.Timer(rospy.Duration(self.sample_time), self.timer_cb)

    def timer_cb(self, event):
        if not self.enabled:
            self.t = 0
            return

        amplitude = 0.5 * (self.high_peak - self.low_peak)
        offset = 0.5 * (self.high_peak + self.low_peak)
        cmd = offset - amplitude * math.cos(2 * math.pi / self.period * self.t)
        self.t += self.sample_time

        self.pub.publish(BrakeCmd(enable=True, pedal_cmd_type=BrakeCmd.CMD_PEDAL, pedal_cmd=cmd))

    def recv_enable(self, msg):
        self.enabled = msg.data


if __name__ == '__main__':
    try:
        node_instance = SineTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

