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

import time
import socket
import json
from struct import pack, unpack

class CCClient:
    # クラス変数
    _SOCKET_IP_ADDRESS = "192.168.0.23"
    _SOCKET_PORT_NUMBER = 5500
    # ロボット操作コマンド
    _CMD_PROGRAM = 0x000
    _CMD_HOME = 0x100
    _CMD_JSPEED = 0x103
    _CMD_P2J = 0x105
    _CMD_QJMOVE = 0x106
    _CMD_SETTOOL = 0x107
    _CMD_SETPOSTURE = 0x10B
    _CMD_GETPOSTURE = 0x10C
    _CMD_ABORTM = 0x10F
    _CMD_SYSSTS = 0x110
    # I/O操作コマンド
    _CMD_SETIO = 0x200
    _CMD_GETIO = 0x201
    _CMD_SETADC = 0x202
    _CMD_GETADC = 0x203
    # その他コマンド
    _CMD_NOCOMMAND = 0xFFF

    _CONNECTING_PROCCES_ROS = 0x02
    # データ種別
    _DATA_TYPE_CMD = 0x00
    _DATA_TYPE_PROGRAM = 0x01
    _DATA_TYPE_CONNECT_CHECK = 0x02
    _DATA_TYPE_OPERATION_GET = 0x03

    _SOCKET_RECV_BUFF_SIZE = 4096

    def __init__(self, ip_address=_SOCKET_IP_ADDRESS):
        # ソケットを作成してデータの送受信を実施
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
        self._sock.connect((ip_address, self._SOCKET_PORT_NUMBER))
        #self.open()

    def _send_data(self, send_data, return_data = {}):
        #print "_send_data execution"
        self._sock.sendall(send_data)
        # ティーチからの送信データを受信
        rec_msg = self._sock.recv(self._SOCKET_RECV_BUFF_SIZE)
        recv_data = json.loads(rec_msg)
        err_code = recv_data["RE"]
        return_data["DA"] = json.loads(recv_data["DA"])
        return err_code

    def _create_send_data(self, cmd_id, data, data_type=_DATA_TYPE_CMD):
        send_json = {
            "CD": cmd_id,
            "PR": self._CONNECTING_PROCCES_ROS,
            "DT": data_type,
            "DA": json.dumps(data)
        }
        send_char_data = json.dumps(send_json)
        pack_format = str(len(send_char_data)) + "s"
        send_data = pack(pack_format, send_char_data)
        return send_data

    def check_connect_status(self):
        data = {}
        send_data = self._create_send_data(self._CMD_NOCOMMAND, data, self._DATA_TYPE_CONNECT_CHECK)
        return_data = self._send_data(send_data)
        return return_data

    def get_operation_permission(self):
        data = {}
        send_data = self._create_send_data(self._CMD_NOCOMMAND, data, self._DATA_TYPE_OPERATION_GET)
        return_data = self._send_data(send_data)
        return return_data

    def exec_program(self, path, param='', delete_flag=0):
        data = {
            "PATH": path,
            "PAR": param,
            "DEL": delete_flag
        }
        send_data = self._create_send_data(self._CMD_PROGRAM, data, self._DATA_TYPE_PROGRAM)
        return_data = self._send_data(send_data)
        return return_data

    def qjmove(self, ax1, ax2, ax3, ax4, ax5, ax6, speed=None):
        print "qjmove execution"
        data = {
            "J1": ax1,
            "J2": ax2,
            "J3": ax3,
            "J4": ax4,
            "J5": ax5,
            "J6": ax6
        }
        if speed is not None:
            data['SP'] = speed
        send_data = self._create_send_data(self._CMD_QJMOVE, data)
        return_data = self._send_data(send_data)
        print "qjmove success"
        print "return:" + str(return_data)
        return return_data

    def abortm(self):
        print "abortm execution"
        data = {}
        send_data = self._create_send_data(self._CMD_ABORTM, data)
        return_data = self._send_data(send_data)
        print "abortm success"
        print "return:" + str(return_data)
        return return_data

    def syssts(self, stat_type, output_data):
        #print "syssts execution"
        data = {
            "TYPE": stat_type
        }
        send_data = self._create_send_data(self._CMD_SYSSTS, data)
        return_data = self._send_data(send_data, output_data)
        #print "syssts success"
        #print "return:" + str(return_data)
        return return_data

    def set_jspeed(self, s):
        print "set_jspeed execution"
        data = {
            "SP": s * 100
        }
        send_data = self._create_send_data(self._CMD_JSPEED, data)
        print "speed設定送信"
        return_data = self._send_data(send_data)
        print "set_jspeed success"
        print "return:" + str(return_data)
        return return_data

    def set_posture(self, posture):
        data = {
            "P": posture
        }
        send_data = self._create_send_data(self._CMD_SETPOSTURE, data)
        return_data = self._send_data(send_data)
        print "set_posture success"
        print "return:" + str(return_data)
        return return_data

    def get_posture(self, output_data):
        data = {
        }
        send_data = self._create_send_data(self._CMD_GETPOSTURE, data)
        return_data = self._send_data(send_data, output_data)
        print "get_posture success"
        print "return:" + str(return_data)
        print "output:" + str(output_data)
        return return_data

    def position_to_joint(self, x, y, z, rx, ry, rz, output_data):
        data = {
            "X": x,
            "Y": y,
            "Z": z,
            "Rx": rx,
            "Ry": ry,
            "Rz": rz,
            "P": -1
        }
        send_data = self._create_send_data(self._CMD_P2J, data)
        return_data = self._send_data(send_data, output_data)
        print "position_to_joint success"
        print "return:" + str(return_data)
        return return_data

    def set_io(self, address, signal):
        data = {
            "AD": address,
            "SL": signal
        }
        send_data = self._create_send_data(self._CMD_SETIO, data)
        return_data = self._send_data(send_data)
        print "set_io success"
        print "return:" + str(return_data)
        return return_data

    def get_io(self, start_addr, end_addr, output_data):
        data = {
            "SA": start_addr,
            "EA": end_addr
        }
        send_data = self._create_send_data(self._CMD_GETIO, data)
        return_data = self._send_data(send_data, output_data)
        print "get_io success"
        print "return:" + str(return_data)
        print "output_data:" + str(output_data)
        return return_data

    def set_adc(self, ch, mode):
        data = {
            "CH": ch,
            "MO": mode
        }
        send_data = self._create_send_data(self._CMD_SETADC, data)
        return_data = self._send_data(send_data)
        print "set_adc success"
        print "return:" + str(return_data)
        return return_data

    def get_adc(self, output_data):
        data = {}
        send_data = self._create_send_data(self._CMD_GETADC, data)
        return_data = self._send_data(send_data, output_data)
        print "get_adc success"
        print "return:" + str(return_data)
        print "output_data:" + str(output_data)
        return return_data

    def close(self):
        self._sock.close()

    def set_tool(self, x, y, z, rz, ry, rx):
        data = {
            "X": x,
            "Y": y,
            "Z": z,
            "Rx": rx,
            "Ry": ry,
            "Rz": rz,
        }
        send_data = self._create_send_data(self._CMD_SETTOOL, data)
        return_data = self._send_data(send_data)
        print "set_tool success"
        print "return:" + str(return_data)
        return return_data

if __name__ == '__main__':
    api = CCClient()
    print "check_connect_status():"
    print api.check_connect_status()
    print "get_operation_permission():"
    print api.get_operation_permission()
    # print api.set_jspeed(0.5)
    # print api.qjmove(30.5, 30.5, 40.5, 50.5, 60.5, 70.5)
    # print api.qjmove(28.4, 28.5, 38.5, 44.5, 60.5, 70.5)
    # #time.sleep(10)
    # output_data = {}
    # print api.set_io(10, "11111")
    # print api.get_io(10, 14, output_data)
    # print "output_data:" + str(output_data)
    # print api.set_adc(1, 1)
    # output_data2 = {}
    # print api.get_adc(output_data2)
    # print "output_data2:" + str(output_data2)