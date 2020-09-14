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
# ---------

import SocketServer
import socket
import os
import time
import math
from simple_message import SimpleMessageSocket, JointPositionMessage, IOStateMessage, StatusMessage, TriStates, RobotMode, WrenchMessage
from force_sensor import ForceSensor
from info_catch_client import InfoCatchClient
import traceback

HOST, PORT = "0.0.0.0", 11002

class StateTCPHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        sock = SimpleMessageSocket(self.request)
        address = self.client_address[0]
        client = InfoCatchClient()
        client.connect(
            [InfoCatchClient.Label.I000,
             InfoCatchClient.Label.R200,
             InfoCatchClient.Label.M000,
             InfoCatchClient.Label.M001,
             InfoCatchClient.Label.M100,
             InfoCatchClient.Label.M102,
             InfoCatchClient.Label.F000,
             InfoCatchClient.Label.F200,
             InfoCatchClient.Label.F300,
             ])
        self._fs = ForceSensor()
        try:
            while True:
                print('receive from shared memory')
                info = client.recv()
                print("send message")
    
                msg = self.make_joint_position(info)
                sock.send(msg)
    
                msg = self.make_io_state(info)
                sock.send(msg)
    
                msg = self.make_robot_status(info)
                sock.send(msg)
    
                msg = self.make_wrench(info)
                if msg:
                    sock.send(msg)
    
        except Exception as e:
            print(e)
            traceback.print_exc()
            client.close()

    def make_joint_position(self, info):
        msg = JointPositionMessage()
        msg.joint_data = info[InfoCatchClient.Label.R200] + [0, 0, 0, 0]
        print(msg.joint_data)
        return msg

    def make_io_state(self, info):
        msg = IOStateMessage()
        msg.digital = info[InfoCatchClient.Label.M000] + info[InfoCatchClient.Label.M001]
        # info catch server doesn't support read analog data
        msg.analog = [0]
        print(msg.digital)
        print(msg.analog)
        return msg

    def make_robot_status(self, info):
        hw_stat = BitAdapter(info[InfoCatchClient.Label.M100][0])
        sw_stat = BitAdapter(info[InfoCatchClient.Label.M102][0])

        servo_on = hw_stat.as_bool(0)
        e_stop = hw_stat.as_bool(1)
        sys_err = sw_stat.as_bool(3)
        stat_code = sw_stat.subseq(4, 4)
        err_code = sw_stat.subseq(8, 8)
        auto_mode = os.path.exists('/tmp/auto_ready')

        msg = StatusMessage()
        msg.mode = RobotMode.AUTO if auto_mode else RobotMode.MANUAL
        msg.e_stopped = TriStates.from_bool(e_stop)
        msg.drives_powered = TriStates.from_bool(servo_on)
        msg.motion_possible = TriStates.from_bool(auto_mode and servo_on and not sys_err)
        msg.in_motion = TriStates.UNKNOWN
        msg.in_error = TriStates.from_bool(sys_err)
        print('{} {} {} {} {} {}'.format(auto_mode, servo_on,
                                            e_stop, sys_err, stat_code, err_code))
        if stat_code in [SysStat.NORMAL_ERROR,
                        SysStat.CRITICAL_ERROR,
                        SysStat.USER_NORMAL_ERROR,
                        SysStat.USER_CRITICAL_ERROR]:
            msg.error_code = stat_code << 8 | err_code
        else:
            msg.error_code = 0

        return msg

    def make_wrench(self, info):
        self._fs.read(info)
        if self._fs.is_valid():
            msg = WrenchMessage()
            msg.force = [self._fs.fx, self._fs.fy, self._fs.fz]
            msg.torque = [self._fs.mx, self._fs.my, self._fs.mz]
            print('wrench: {} {}'.format(msg.force, msg.torque))
            return msg
        else:
            return None

class SysStat:
    BOOT = 0
    INIT = 1
    READY = 2
    INC = 3
    TEACH = 4
    JOG = 5
    RUN = 6
    PAUSE = 7
    NORMAL_ERROR = 10
    CRITICAL_ERROR = 11
    USER_NORMAL_ERROR = 12
    USER_CRITICAL_ERROR = 13

class BitAdapter:
    def __init__(self, num):
        self._num = num

    def as_bool(self, n):
        return bool((self._num >> n) & 1)

    def subseq(self, n, bit_len):
        mask = (1 << (n + bit_len)) - 1
        return (self._num & mask) >> n

class StateServer(SocketServer.ThreadingTCPServer, object):
    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)

if __name__ == "__main__":
    server = StateServer((HOST, PORT), StateTCPHandler)
    server.serve_forever()
