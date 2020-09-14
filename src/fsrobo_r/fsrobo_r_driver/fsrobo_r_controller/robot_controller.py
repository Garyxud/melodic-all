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

from cc_client import CCClient

class RobotController:
    def __init__(self, permission = True):
        self._api = CCClient()
        if permission:
            self._api.get_operation_permission()

    def exec_program(self, path, param=''):
        return self._api.exec_program(path, param) == 0

    def set_speed(self, speed):
        return self._api.set_jspeed(speed) == 0

    def set_posture(self, posture):
        return self._api.set_posture(posture) == 0
    
    def get_posture(self):
        data = {}
        result = self._api.get_posture(data)
        if result == 0:
            return data['DA']['P']
        else:
            return None

    def move(self, joint, speed=None):
        if speed is None:
            params = joint
        else:
            params = joint + [speed]
        return self._api.qjmove(*params) == 0

    def abort(self):
        return self._api.abortm() == 0

    def sys_stat(self, stat_type):
        data = {}
        result = self._api.syssts(stat_type, data)
        if result == 0:
            return data['DA']['RE']
        else:
            return None

    def set_tool_offset(self, x, y, z, rz, ry, rx):
        result = self._api.set_tool(x, y, z, rz, ry, rx)

        return result == 0

    def set_dio(self, addr, data):
        def to_char(x):
            if x == -1:
                return '*'
            else:
                return str(x)
        io_list = map(to_char, data)
        io_list.reverse()
        io_str = ''.join(io_list)
        return self._api.set_io(addr, io_str) == 0
    
    def get_dio(self, start_addr, end_addr):
        data = {}
        result = self._api.get_io(start_addr, end_addr, data)
        if result == 0:
            io_str = data['DA']['SL']
            io_list = list(io_str)
            io_list.reverse()
            return io_list
        else:
            return []

    def set_adc_mode(self, ch, mode):
        return self._api.set_adc(ch, mode) == 0

    def close(self):
        self._api.close()

if __name__ == '__main__':
    ctl = RobotController()
    print ctl.set_speed(100)
    print ctl.move([0, 0, 0, 0, 0, 0])
    print ctl.move([22, 23, 24, 30, 33, 10])
    print ctl.set_dio(0, [0, 0, 0, 0, 0])
    print ctl.get_dio(0, 31)
    print ctl.set_dio(0, [1, -1, 1, 0, 0])
    print ctl.get_dio(0, 31)
    print ctl.get_posture()
    print ctl.set_posture(4)
    print ctl.get_posture()