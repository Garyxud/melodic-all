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
from simple_message import SimpleMessageSocket, \
    SimpleMessageType, JointTrajPtReplyMessage, ReplyCode, SpecialSequence, \
    SetIOReplyMessage, IOFunctionType, \
    SetPostureReplyMessage, GetPostureReplyMessage, \
    SysStatReplyMessage, \
    SetToolOffsetReplyMessage
from robot_controller import RobotController

HOST, PORT = "0.0.0.0", 11003

class RobotTCPHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        #client = self.request
        sock = SimpleMessageSocket(self.request)
        address = self.client_address[0]
        print('connected from {}'.format(address))
        ctl = RobotController()

        msg_handlers = {
            SimpleMessageType.SET_IO: self.on_set_io,
            SimpleMessageType.SET_POSTURE: self.on_set_posture,
            SimpleMessageType.GET_POSTURE: self.on_get_posture,
            SimpleMessageType.SYS_STAT: self.on_sys_stat,
            SimpleMessageType.SET_TOOL_OFFSET: self.on_set_tool_offset
        }

        try:
            while True:
                recv_msg = sock.recv()

                msg_handler = msg_handlers.get(
                    recv_msg.msg_type, self.on_unkown_message)

                reply_msg = msg_handler(ctl, recv_msg)
                sock.send(reply_msg)
        finally:
            ctl.close()

    def on_set_io(self, ctl, recv_msg):
        print(recv_msg.fun)
        print(recv_msg.address)
        print(recv_msg.data_size)
        print(recv_msg.data)

        if recv_msg.fun == IOFunctionType.SET_DIGITAL_OUT:
            result = ctl.set_dio(
                recv_msg.address, recv_msg.data[0:recv_msg.data_size])
        elif recv_msg.fun == IOFunctionType.SET_ADC_MODE:
            result = ctl.set_adc_mode(recv_msg.address, recv_msg.data[0])

        reply_msg = SetIOReplyMessage()
        reply_msg.reply_code = ReplyCode.SUCCESS if result else ReplyCode.FAILURE
        reply_msg.result = reply_msg.Result.SUCCESS if result else reply_msg.Result.FAILURE
        print("received: I/O message")

        return reply_msg

    def on_set_posture(self, ctl, recv_msg):
        print(recv_msg.posture)
        print("received: Set posture message")

        result = ctl.set_posture(recv_msg.posture)

        reply_msg = SetPostureReplyMessage()
        reply_msg.reply_code = ReplyCode.SUCCESS if result else ReplyCode.FAILURE

        return reply_msg

    def on_get_posture(self, ctl, recv_msg):
        print("received: Get posture message")

        posture = ctl.get_posture()

        reply_msg = GetPostureReplyMessage()
        if posture == None:
            reply_msg.reply_code = ReplyCode.FAILURE
        else:
            reply_msg.reply_code = ReplyCode.SUCCESS
            reply_msg.posture = posture

        return reply_msg

    def on_sys_stat(self, ctl, recv_msg):
        print(recv_msg.stat_type)

        result = ctl.sys_stat(recv_msg.stat_type)

        reply_msg = SysStatReplyMessage()
        reply_msg.reply_code = ReplyCode.SUCCESS if result else ReplyCode.FAILURE
        print("result: {}".format(result))
        reply_msg.result = result
        print("received: sys_stat message")

        return reply_msg

    def on_set_tool_offset(self, ctl, recv_msg):
        print("received: Set tool offset message")
        result = ctl.set_tool_offset(recv_msg.x, recv_msg.y, recv_msg.z,
            recv_msg.rz, recv_msg.ry, recv_msg.rx)
        print("result: {}".format(result))
        
        reply_msg = SetToolOffsetReplyMessage()
        reply_msg.reply_code = SetToolOffsetReplyMessage.Result.SUCCESS if result else SetToolOffsetReplyMessage.Result.FAILURE

        return reply_msg

    def on_unkown_message(self, ctl, recv_msg):
        raise NotImplementedError(
            "Unknown msg_type: {}".format(recv_msg.msg_type))


class RobotServer(SocketServer.ForkingTCPServer, object):
    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)


if __name__ == "__main__":
    server = RobotServer((HOST, PORT), RobotTCPHandler)
    server.serve_forever()
