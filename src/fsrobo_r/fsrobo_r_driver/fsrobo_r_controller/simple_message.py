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

from struct import pack,unpack

class TriStates:
    UNKNOWN = -1
    ON = TRUE = ENABLED = HIGH = 1
    OFF = FALSE = DISABLED = LOW = 0

    @classmethod
    def from_bool(self, b):
        return self.TRUE if b else self.FALSE

class SpecialSequence:
    START_TRAJECTORY_DOWNLOAD = -1
    START_TRAJECTORY_STREAMING = -2
    END_TRAJECTORY = -3
    STOP_TRAJECTORY = -4

class SimpleMessageType:
    JOINT_POSITION = 10
    JOINT_TRAJ_PT = 11
    STATUS = 13
    SET_IO = 9001
    IO_STATE = 9003
    EXECUTE_PROGRAM = 9004
    SET_POSTURE = 9005
    GET_POSTURE = 9006
    WRENCH = 9007
    SYS_STAT = 9008
    SET_TOOL_OFFSET = 9009

class CommunicationType:
    INVALID = 0
    TOPIC = 1
    SERVICE_REQUEST = 2
    SERVICE_REPLY = 3

class ReplyCode:
    INVALID = UNUSED = 0
    SUCCESS = 1
    FAILURE = 2

class RobotMode:
    UNKNOWN = -1
    MANUAL = 1
    AUTO = 2

class IOFunctionType:
    SET_DIGITAL_OUT = 1
    SET_ADC_MODE = 2

class IOState:
    NO_CHANGE = -1
    ON = 1
    OFF = 0

class SimpleMessage(object):
    PREFIX_SIZE = 4
    HEADER_SIZE = 12
    # Prefix
    length = 0
    # Header
    msg_type = 0
    comm_type = 0
    reply_code = 0
    body = []

    def make_body(self):
        return ""

    def load_body(self, body):
        pass

    def dump(self):
        body = self.make_body()
        prefix = pack("i", SimpleMessage.HEADER_SIZE + len(body))
        header = pack("3i", self.msg_type, self.comm_type, self.reply_code)
        return prefix + header + self.make_body()

    def load(self, data):
        self.length, self.msg_type, self.comm_type, self.reply_code = unpack('4i', data[:16])
        self.load_body(data[16:])
 

def recvBytes(socket, num):
    MAX_SIZE = 4096

    remain = num
    data = ""

    while remain > 0:
        print("recv: {}".format(remain))
        buf = socket.recv(MAX_SIZE if remain > MAX_SIZE else remain)

        buf_size = len(buf)

        if buf_size == 0: #　TODO handling error
            assert()

        remain -= buf_size
        data += buf

    return data

def sendBytes(socket, buf):
    buf_size = len(buf)
    remain = buf_size
    
    while remain > 0:
        print("send: {}".format(remain))
        i = buf_size - remain
        send_size = socket.send(buf[i:])

        if send_size == 0: # TODO handling error
            assert()
            
        remain -= send_size

class JointPositionMessage(SimpleMessage):
    "TODO docstring"
    sequence = 0

    def __init__(self):
        self.msg_type = SimpleMessageType.JOINT_POSITION
        self.comm_type = CommunicationType.TOPIC
        self.joint_data = [0] * 10

    def make_body(self):
        return pack("i10f", self.sequence, *self.joint_data)

class JointTrajPtMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.JOINT_TRAJ_PT
        self.comm_type = CommunicationType.SERVICE_REQUEST
        self.sequence = 0
        self.joint_data = [0] * 10
        self.velocity = 0
        self.duration = 0

    def make_body(self):
        return pack('i10f', self.sequence, *self.joint_data) \
               + pack('2f', self.velocity, self.duration)

    def load_body(self, body):
        self.sequence, = unpack('i', body[0:4])
        self.joint_data = list(unpack('10f', body[4:44]))
        self.velocity, self.duration = unpack('2f', body[44:52])

class JointTrajPtReplyMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.JOINT_TRAJ_PT
        self.comm_type = CommunicationType.SERVICE_REPLY
        self.sequence = 0
        self.joint_data = [0] * 10

    def make_body(self):
        return pack('i10f', self.sequence, *self.joint_data)

    def load_body(self, body):
        self.sequence, = unpack('i', body[0:4])
        self.joint_data = list(unpack('10f', body[4:44]))

class StatusMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.STATUS
        self.comm_type = CommunicationType.TOPIC
        self.mode = RobotMode.UNKNOWN
        self.e_stopped = TriStates.UNKNOWN
        self.drives_powered = TriStates.UNKNOWN
        self.motion_possible = TriStates.UNKNOWN
        self.in_motion = TriStates.UNKNOWN
        self.in_error = TriStates.UNKNOWN
        self.error_code = 0

    def make_body(self):
        return pack("7i",
                    self.drives_powered,
                    self.e_stopped,
                    self.error_code,
                    self.in_error,
                    self.in_motion,
                    self.mode,
                    self.motion_possible)

class SetIOMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = 9001
        self.comm_type = CommunicationType.SERVICE_REQUEST
        self.fun = 0
        self.address = 0
        self.data_size = 0
        self.data = [0] * 32

    def make_body(self):
        return pack('ii32i', self.fun, self.address, *self.data)

    def load_body(self, body):
        self.fun, self.address, self.data_size = unpack('3i', body[0:12])
        self.data = list(unpack('32i', body[12:140]))

class SetIOReplyMessage(SimpleMessage):
    "TODO docstring"

    class Result:
        FAILURE = 0
        SUCCESS = 1

    def __init__(self):
        self.msg_type = 9002
        self.comm_type = CommunicationType.SERVICE_REPLY
        self.result = self.Result.SUCCESS

    def make_body(self):
        return pack('i', self.result)

    def load_body(self, body):
        self.result = unpack('i', body[0:4])

class IOStateMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.IO_STATE
        self.comm_type = CommunicationType.TOPIC
        self.digital = [0] * 2
        self.analog = [0]

    def make_body(self):
        return pack("2i", *self.digital) + pack("i", *self.analog)

class WrenchMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.WRENCH
        self.comm_type = CommunicationType.TOPIC
        self.force = [0] * 3
        self.torque = [0] * 3

    def make_body(self):
        return pack("3f", *self.force) + pack("3f", *self.torque)


class ExecuteProgramMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.EXECUTE_PROGRAM
        self.comm_type = CommunicationType.SERVICE_REQUEST
        self.name = ''
        self.param = ''

    def make_body(self):
        return self.name + pack('i', len(self.name))

    def load_body(self, body):
        param_size, = unpack('i', body[-4:])
        self.param = body[-(param_size + 4):-4]
        name_size, = unpack('i', body[-(param_size + 4 * 2):-(param_size + 4)])
        self.name = body[0:name_size]

class ExecuteProgramReplyMessage(SimpleMessage):
    "TODO docstring"

    class Result:
        FAILURE = 0
        SUCCESS = 1

    def __init__(self):
        self.msg_type = SimpleMessageType.EXECUTE_PROGRAM
        self.comm_type = CommunicationType.SERVICE_REPLY
        self.result = self.Result.SUCCESS

    def make_body(self):
        return pack('i', self.result)

    def load_body(self, body):
        self.result, = unpack('i', body[0:4])

class SetPostureMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.SET_POSTURE
        self.comm_type = CommunicationType.SERVICE_REQUEST
        self.posture = 0

    def make_body(self):
        return pack('i', self.posture)

    def load_body(self, body):
        self.posture, = unpack('i', body[0:4])

class SetPostureReplyMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.SET_POSTURE
        self.comm_type = CommunicationType.SERVICE_REPLY
        self.posture = 0

    def make_body(self):
        return pack('i', self.posture)

    def load_body(self, body):
        self.posture, = unpack('i', body[0:4])

class GetPostureMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.GET_POSTURE
        self.comm_type = CommunicationType.SERVICE_REQUEST
        self.posture = 0

    def make_body(self):
        return pack('i', self.posture)

    def load_body(self, body):
        self.posture, = unpack('i', body[0:4])

class GetPostureReplyMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.GET_POSTURE
        self.comm_type = CommunicationType.SERVICE_REPLY
        self.posture = 0

    def make_body(self):
        return pack('i', self.posture)

    def load_body(self, body):
        self.posture, = unpack('i', body[0:4])

class SysStatMessage(SimpleMessage):
    "TODO docstring"

    class StatType:
        EMS_STATE = 1
        MOTION_ID = 2
        INC_MODE = 3
        ABS_LOST = 4
        MOTION_REQUEST = 5
        MOTION_STATE = 6
        SPI_ERROR = 7
        SERVO_CONTROL = 8
        PULSE = 0x8000

    def __init__(self):
        self.msg_type = SimpleMessageType.SYS_STAT
        self.comm_type = CommunicationType.SERVICE_REQUEST
        self.stat_type = 0
        self.result = 0

    def make_body(self):
        return pack('2i', self.stat_type, self.result)

    def load_body(self, body):
        self.stat_type, self.result = unpack('2i', body[0:8])

class SysStatReplyMessage(SimpleMessage):
    "TODO docstring"

    class StatType:
        EMS_STATE = 1
        MOTION_ID = 2
        INC_MODE = 3
        ABS_LOST = 4
        MOTION_REQUEST = 5
        MOTION_STATE = 6
        SPI_ERROR = 7
        SERVO_CONTROL = 8
        PULSE = 0x8000

    def __init__(self):
        self.msg_type = SimpleMessageType.SYS_STAT
        self.comm_type = CommunicationType.SERVICE_REPLY
        self.stat_type = 0
        self.result = 0

    def make_body(self):
        return pack('2i', self.stat_type, self.result)

    def load_body(self, body):
        self.stat_type, self.result = unpack('2i', body[0:8])

class SetToolOffsetMessage(SimpleMessage):
    "TODO docstring"

    def __init__(self):
        self.msg_type = SimpleMessageType.SET_TOOL_OFFSET
        self.comm_type = CommunicationType.SERVICE_REQUEST
        self.x = 0
        self.y = 0
        self.z = 0
        self.rz = 0
        self.rx = 0
        self.ry = 0

    def make_body(self):
        return pack('6f', self.x, self.y, self.z, self.rz, self.ry, self.rx)

    def load_body(self, body):
        self.x, self.y, self.z, self.rz, self.ry, self.rx = unpack('6f', body)

class SetToolOffsetReplyMessage(SimpleMessage):
    "TODO docstring"

    class Result:
        FAILURE = 0
        SUCCESS = 1

    def __init__(self):
        self.msg_type = SimpleMessageType.SET_TOOL_OFFSET
        self.comm_type = CommunicationType.SERVICE_REPLY
        self.result = self.Result.SUCCESS

    def make_body(self):
        return pack('i', self.result)

    def load_body(self, body):
        self.result = unpack('i', body[0:4])


class SimpleMessageSocket:
    MAX_BUFFER_SIZE = 4096

    def __init__(self, sock):
        self._sock = sock

    def send(self, message):
        self._send_bytes(message.dump())

    def recv(self):
        data = self._recv_message_bytes()
        #print("{:02x}".format(ord(data[0])))
        print(":".join("{:02x}".format(ord(c)) for c in data))
        prefix, msg_type = unpack("2i", data[0:8])

        msg_class = {
            SimpleMessageType.JOINT_TRAJ_PT: JointTrajPtMessage,
            SimpleMessageType.STATUS: StatusMessage,
            SimpleMessageType.SET_IO: SetIOMessage,
            SimpleMessageType.EXECUTE_PROGRAM: ExecuteProgramMessage,
            SimpleMessageType.SET_POSTURE: SetPostureMessage,
            SimpleMessageType.GET_POSTURE: GetPostureMessage,
            SimpleMessageType.SYS_STAT: SysStatMessage,
            SimpleMessageType.SET_TOOL_OFFSET: SetToolOffsetMessage,
        }.get(msg_type, SimpleMessage)

        msg = msg_class()
        msg.load(data)

        return msg

    def _recv_message_bytes(self):
        prefix = self._recv_bytes(4)
        data_size, = unpack("I", prefix)
        data = self._recv_bytes(data_size)

        return prefix + data

    def _recv_bytes(self, num):
        remain = num
        data = ""

        while remain > 0:
            print("recv: {}".format(remain))
            buf = self._sock.recv(self.MAX_BUFFER_SIZE if remain > self.MAX_BUFFER_SIZE else remain)

            buf_size = len(buf)

            if buf_size == 0: # TODO handling error
                assert()

            remain -= buf_size
            data += buf

        return data

    def _send_bytes(self, data):

        data_size = len(data)
        remain = data_size

        while remain > 0:
            print("send: {}".format(remain))
            i = data_size - remain
            send_size = self._sock.send(data[i:])

            if send_size == 0: # TODO handling error
                assert()

            remain -= send_size
