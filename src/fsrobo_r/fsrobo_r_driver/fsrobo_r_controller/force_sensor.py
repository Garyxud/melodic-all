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

from info_catch_client import InfoCatchClient

class ForceSensor(object):
    MA_BUFFER_SIZE = 4
    class Status(object):
        NO_IF           = 0x00    # インターフェイス基板なし
        VALID           = 0x1F    # 力覚センサ有効
        ERR_MASK        = 0x17    # エラー判定のためのマスク
        ERR_NO_RESPONSE = 0x10    # 力覚センサ応答なし
        ERR_FRAME_1     = 0x11    # フレームズレ１
        ERR_FRAME_2     = 0x12    # フレームズレ１
        ERR_DATA        = 0x13    # データ異常

    def __init__(self):
        self._fx = 0
        self._fy = 0
        self._fz = 0
        self._mx = 0
        self._my = 0
        self._mz = 0
        self._ma_buf = [[0.0] * 6] * self.MA_BUFFER_SIZE
        self._ma_index = 0

    def read(self, info):
        self._status = info[InfoCatchClient.Label.F000][0]
        #self._zero_point = [float(x) for x in shm_read(0x0710,6).split(',')]
        self._zero_point = [8192] * 6
        self._raw_val = info[InfoCatchClient.Label.F200][0:6]
        self._gain = info[InfoCatchClient.Label.F300][0:6]
        #self._status = int(shm_read(0x1800,1))
        #self._zero_point = [float(x) for x in shm_read(0x1c00,6).split(',')]
        #self._raw_val = [float(x) for x in shm_read(0x1d00,6).split(',')]
        #self._gain = [float(x) for x in shm_read(0x1e00,6).split(',')]


        if all(self._gain):
            val = []
            for i in range(6):
                val.append((self._raw_val[i] - self._zero_point[i]) / self._gain[i])
            ma_val = self._calc_ma(val)
            self._fx = ma_val[0]
            self._fy = ma_val[1]
            self._fz = ma_val[2]
            self._mx = ma_val[3]
            self._my = ma_val[4]
            self._mz = ma_val[5]

    def _calc_ma(self, val):
        self._ma_buf[self._ma_index] = val
        ma_val = []

        for i in range(6):
            avg = sum([x[i] for x in self._ma_buf]) / self.MA_BUFFER_SIZE
            ma_val.append(avg)

        self._ma_index = (self._ma_index + 1) % self.MA_BUFFER_SIZE

        return ma_val


    def is_valid(self):
        return self._status == self.Status.VALID

    @property
    def raw_fx(self):
        return self._raw_val[0]

    @property
    def raw_fy(self):
        return self._raw_val[1]

    @property
    def raw_fz(self):
        return self._raw_val[2]

    @property
    def raw_mx(self):
        return self._raw_val[3]

    @property
    def raw_my(self):
        return self._raw_val[4]

    @property
    def raw_mz(self):
        return self._raw_val[5]

    @property
    def fx(self):
        return self._fx

    @property
    def fy(self):
        return self._fy

    @property
    def fz(self):
        return self._fz

    @property
    def mx(self):
        return self._mx

    @property
    def my(self):
        return self._my

    @property
    def mz(self):
        return self._mz


if __name__ == '__main__':
    fs = ForceSensor()
    fs.read()
    print('F x: {} y: {} z: {}'.format(fs.fx, fs.fy, fs.fz))
    print('M x: {} y: {} z: {}'.format(fs.mx, fs.my, fs.mz))