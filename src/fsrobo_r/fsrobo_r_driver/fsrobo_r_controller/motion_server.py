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
import threading
import Queue
from simple_message import SimpleMessageSocket, \
    SimpleMessageType, JointTrajPtReplyMessage, ReplyCode, \
    SpecialSequence, ExecuteProgramReplyMessage
from robot_controller import RobotController
import math

HOST, PORT = "0.0.0.0", 11000

class MotionThread(threading.Thread):
    def __init__(self, ctl):
        super(MotionThread, self).__init__()
        self._ctl = ctl
        self._q_move = Queue.Queue()
        self._q_abort = Queue.Queue()
        self.daemon = True

    def run(self):
        while True:
            try:
                self._q_abort.get(True, 0.001)
                self._ctl.abort()
                with self._q_move.mutex:
                    self._q_move.queue.clear()
            except Queue.Empty:
                pass
            used_buffer_size = self._ctl.sys_stat(5)
            if used_buffer_size < 4:
                try:
                    joints, speed = self._q_move.get(False)
                    self._ctl.move(joints, speed)
                except Queue.Empty:
                    pass

    def move(self, joints, speed):
        self._q_move.put((joints, speed))

    def abort(self):
        self._q_abort.put(True)

class MotionTCPHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        #client = self.request
        sock = SimpleMessageSocket(self.request)
        address = self.client_address[0]
        print('connected with {}'.format(address))
        ctl = RobotController()
        motion_thread = MotionThread(ctl)
        motion_thread.start()

        msg_handlers = {
            SimpleMessageType.JOINT_TRAJ_PT: self.on_joint_traj_pt,
            SimpleMessageType.EXECUTE_PROGRAM: self.on_execute_program
        }

        try:
            while True:
                recv_msg = sock.recv()

                msg_handler = msg_handlers.get(
                    recv_msg.msg_type, self.on_unkown_message)

                reply_msg = msg_handler(ctl, recv_msg, motion_thread)
                sock.send(reply_msg)
        finally:
            ctl.close()

    def on_joint_traj_pt(self, ctl, recv_msg, motion_thread):
        joint_deg = map(math.degrees, recv_msg.joint_data[:6])
        #print(recv_msg.sequence)
        #print(recv_msg.joint_data)

        if recv_msg.sequence == SpecialSequence.STOP_TRAJECTORY:
            motion_thread.abort()
            print("abort move command")
        else:
            motion_thread.move(joint_deg, recv_msg.velocity * 100)

        reply_msg = JointTrajPtReplyMessage()
        reply_msg.reply_code = ReplyCode.SUCCESS

        return reply_msg

    def on_execute_program(self, ctl, recv_msg, motion_thread):
        print(recv_msg.name)
        print(recv_msg.param)
        print("received: Execute program message")

        result = ctl.exec_program(recv_msg.name, recv_msg.param)

        reply_msg = ExecuteProgramReplyMessage()
        reply_msg.reply_code = ReplyCode.SUCCESS
        reply_msg.result = reply_msg.Result.SUCCESS if result else reply_msg.Result.FAILURE

        return reply_msg

    def on_unkown_message(self, ctl, recv_msg, motion_thread):
        raise NotImplementedError(
            "Unknown msg_type: {}".format(recv_msg.msg_type))


class MotionServer(SocketServer.ForkingTCPServer, object):
    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)


if __name__ == "__main__":
    server = MotionServer((HOST, PORT), MotionTCPHandler)
    server.serve_forever()
