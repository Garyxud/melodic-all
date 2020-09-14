#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Dataspeed Inc.
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
from dbw_mkz_msgs.msg import WheelPositionReport, FuelLevelReport
from math import fabs

class WheelCountsPerKm:
    def __init__(self):
        rospy.init_node('wheel_counts_per_km')
        
        # Variables
        self.wheel_position_old = 0
        self.wheel_position_old_ready = False
        self.msg_fuel_level = FuelLevelReport()
        self.msg_fuel_level_ready = False
        self.msg_wheel_position = WheelPositionReport()
        self.msg_wheel_position_ready = False
        
        # Subscribers
        rospy.Subscriber('/vehicle/fuel_level_report', FuelLevelReport, self.recv_fuel_level)
        rospy.Subscriber('/vehicle/wheel_position_report', WheelPositionReport, self.recv_wheel_position)

    def recv_fuel_level(self, msg):
        if self.msg_fuel_level_ready:
            if int(msg.odometer) == int(self.msg_fuel_level.odometer + 1):
                if self.wheel_position_old_ready:
                    diff = self.msg_wheel_position.rear_left - self.wheel_position_old.rear_left
                    if diff < 0:
                        diff += 65536
                    rospy.loginfo('Odometer km increment: '  + str(diff) + ' wheel position counts.')
                else:
                    rospy.loginfo('Odometer km increment: (first)')
                if self.msg_wheel_position_ready:
                    self.wheel_position_old = self.msg_wheel_position
                    self.wheel_position_old_ready = True
        self.msg_fuel_level = msg
        self.msg_fuel_level_ready = True

    def recv_wheel_position(self, msg):
        self.msg_wheel_position = msg
        self.msg_wheel_position_ready = True

if __name__ == '__main__':
    try:
        node = WheelCountsPerKm()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

