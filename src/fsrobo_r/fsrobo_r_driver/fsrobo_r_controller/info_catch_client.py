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

class JSONSocket(object):
    BUFFER_SIZE = 4096

    def __init__(self, ip_addr, port):
        self._ip_addr = ip_addr
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._decoder = json.JSONDecoder(strict=False)
        self._recv_buffer = ''

    def connect(self):
        self._sock.connect((self._ip_addr, self._port))

    def close(self):
        self._sock.close()

    def send(self, data):
        self._sock.sendall(json.dumps(data))

    def recv(self):
        need_recv = len(self._recv_buffer) == 0
        #print('len: {}'.format(len(self._recv_buffer)))

        while True:
            try:
                if need_recv:
                    recv_data = self._sock.recv(self.BUFFER_SIZE)
                    if (len(recv_data) == 0):
                        raise socket.error('recv error')

                    self._recv_buffer += recv_data
                    need_recv = False
                else:
                    # XXX leading null char causes ValueError. Should fix server?
                    self._recv_buffer = self._recv_buffer.strip('\0')

                    data, index = self._decoder.raw_decode(self._recv_buffer)
                    self._recv_buffer = self._recv_buffer[index:]
                    #print('OK!:{}:{}:'.format(self._recv_buffer, self._recv_buffer.encode('hex')))
                    return data
            except ValueError as e:
                #print(e)
                #print(self._recv_buffer)
                #print(self._recv_buffer.encode('hex'))
                need_recv = True

class InfoCatchClient(object):
    ControlPort = 5000

    def __init__(self, ip_addr='192.168.0.23'):
        self._ip_addr = ip_addr
        self._data_port = None
        self._data_sock = None

    def send_control(self, msg):
        sock = JSONSocket(self._ip_addr, self.ControlPort)
        sock.connect()

        sock.send(msg)
        data = sock.recv()

        sock.close()

        return data

    def recv(self):
        return self._data_sock.recv()

    def connect(self, filters, sampling_time=10):
        control_data = {'ST00': 'ON', 'ST01': sampling_time, 'ST02': sampling_time}

        for filter in filters:
            control_data[filter] = 'ON'

        res = self.send_control(control_data)
        if (res['RT00'] != 'OK'):
            return False

        self._data_port = res['RT01']
        self._data_sock = JSONSocket(self._ip_addr, self._data_port)

        retry_count = 0
        while True:
            try:
                self._data_sock.connect()
                break
            except socket.error as e:
                retry_count += 1
                if retry_count > 10:
                    raise socket.error(e)
                time.sleep(0.5)

    def close(self):
        if self._data_port is not None:
            control_data = {'ST00': 'OFF', 'RT01': self._data_port}
            res = self.send_control(control_data)
            if res['RT00'] != 'OK':
                print('warning: send_control returns {}'.format(res))
            self._data_port = None

        if self._data_sock is not None:
            self._data_sock.close()

    class Label(object):
        I000 = "i000" # 0xFFFF Date/Time
        # Header block
        H003 = "h003" # 0x0008 update_counter
        H004 = "h004" # 0x000C now_updating
        # Memory I/O
        M000 = "m000" # 0x0100 dio_io
        M001 = "m001" # 0x0104 dio_io
        M100 = "m100" # 0x0300 mio_si0
        M102 = "m102" # 0x0308 mio_si2
        M107 = "m107" # 0x031C mio_sl3
        M200 = "m200" # 0x0320 dio_io[0]
        M201 = "m201" # 0x0324 dio_io[1]
        M202 = "m202" # 0x0328 dio_io[2]
        M203 = "m203" # 0x032C dio_io[3]
        M204 = "m204" # 0x0330 dio_io[4]
        M205 = "m205" # 0x0334 dio_io[5]
        M206 = "m206" # 0x0338 dio_io[6]
        M207 = "m207" # 0x033C dio_io[7]
        M208 = "m208" # 0x0340 dio_io[8]
        M209 = "m209" # 0x0344 dio_io[9]
        M210 = "m210" # 0x0348 dio_io[10]
        M211 = "m211" # 0x034C dio_io[11]
        M212 = "m212" # 0x0350 dio_io[12]
        M213 = "m213" # 0x0354 dio_io[13]
        M214 = "m214" # 0x0358 dio_io[14]
        M215 = "m215" # 0x035C dio_io[15]
        M216 = "m216" # 0x0360 dio_io[16]
        M217 = "m217" # 0x0364 dio_io[17]
        M218 = "m218" # 0x0368 dio_io[18]
        M219 = "m219" # 0x036C dio_io[19]
        M220 = "m220" # 0x0370 dio_io[20]
        M221 = "m221" # 0x0374 dio_io[21]
        M222 = "m222" # 0x0378 dio_io[22]
        M223 = "m223" # 0x037C dio_io[23]
        M224 = "m224" # 0x0380 dio_io[24]
        M225 = "m225" # 0x0384 dio_io[25]
        M226 = "m226" # 0x0388 dio_io[26]
        M227 = "m227" # 0x038C dio_io[27]
        M228 = "m228" # 0x0390 dio_io[28]
        M229 = "m229" # 0x0394 dio_io[29]
        M230 = "m230" # 0x0398 dio_io[30]
        M231 = "m231" # 0x039C dio_io[31]
        M232 = "m232" # 0x03A0 dio_io[32]
        M233 = "m233" # 0x03A4 dio_io[33]
        M234 = "m234" # 0x03A8 dio_io[34]
        M235 = "m235" # 0x03AC dio_io[35]
        M236 = "m236" # 0x03B0 dio_io[36]
        M237 = "m237" # 0x03B4 dio_io[37]
        M238 = "m238" # 0x03B8 dio_io[38]
        M239 = "m239" # 0x03BC dio_io[39]
        M240 = "m240" # 0x03C0 dio_io[40]
        M241 = "m241" # 0x03C4 dio_io[41]
        M242 = "m242" # 0x03C8 dio_io[42]
        M243 = "m243" # 0x03CC dio_io[43]
        M244 = "m244" # 0x03D0 dio_io[44]
        M245 = "m245" # 0x03D4 dio_io[45]
        M246 = "m246" # 0x03D8 dio_io[46]
        M247 = "m247" # 0x03DC dio_io[47]
        M248 = "m248" # 0x03E0 dio_io[48]
        M249 = "m249" # 0x03E4 dio_io[49]
        M250 = "m250" # 0x03E8 dio_io[50]
        M251 = "m251" # 0x03EC dio_io[51]
        M252 = "m252" # 0x03F0 dio_io[52]
        M253 = "m253" # 0x03F4 dio_io[53]
        M254 = "m254" # 0x03F8 dio_io[54]
        M255 = "m255" # 0x03FC dio_io[55]
        M256 = "m256" # 0x0400 dio_io[56]
        M257 = "m257" # 0x0404 dio_io[57]
        M258 = "m258" # 0x0408 dio_io[58]
        M259 = "m259" # 0x040C dio_io[59]
        M260 = "m260" # 0x0410 dio_io[60]
        M261 = "m261" # 0x0414 dio_io[61]
        M262 = "m262" # 0x0418 dio_io[62]
        M263 = "m263" # 0x041C dio_io[63]
        M264 = "m264" # 0x0420 dio_io[64]
        M265 = "m265" # 0x0424 dio_io[65]
        M266 = "m266" # 0x0428 dio_io[66]
        M267 = "m267" # 0x042C dio_io[67]
        M268 = "m268" # 0x0430 dio_io[68]
        M269 = "m269" # 0x0434 dio_io[69]
        M270 = "m270" # 0x0438 dio_io[70]
        M271 = "m271" # 0x043C dio_io[71]
        M272 = "m272" # 0x0440 dio_io[72]
        M273 = "m273" # 0x0444 dio_io[73]
        M274 = "m274" # 0x0448 dio_io[74]
        M275 = "m275" # 0x044C dio_io[75]
        M276 = "m276" # 0x0450 dio_io[76]
        M277 = "m277" # 0x0454 dio_io[77]
        M278 = "m278" # 0x0458 dio_io[78]
        M279 = "m279" # 0x045C dio_io[79]
        M280 = "m280" # 0x0460 dio_io[80]
        M281 = "m281" # 0x0464 dio_io[81]
        M282 = "m282" # 0x0468 dio_io[82]
        M283 = "m283" # 0x046C dio_io[83]
        M284 = "m284" # 0x0470 dio_io[84]
        M285 = "m285" # 0x0474 dio_io[85]
        M286 = "m286" # 0x0478 dio_io[86]
        M287 = "m287" # 0x047C dio_io[87]
        M288 = "m288" # 0x0480 dio_io[88]
        M289 = "m289" # 0x0484 dio_io[89]
        M290 = "m290" # 0x0488 dio_io[90]
        M291 = "m291" # 0x048C dio_io[91]
        M292 = "m292" # 0x0490 dio_io[92]
        M293 = "m293" # 0x0494 dio_io[93]
        M294 = "m294" # 0x0498 dio_io[94]
        M295 = "m295" # 0x049C dio_io[95]
        M296 = "m296" # 0x04A0 dio_io[96]
        M297 = "m297" # 0x04A4 dio_io[97]
        M298 = "m298" # 0x04A8 dio_io[98]
        M299 = "m299" # 0x04AC dio_io[99]
        M300 = "m300" # 0x04B0 dio_io[100]
        M301 = "m301" # 0x04B4 dio_io[101]
        M302 = "m302" # 0x04B8 dio_io[102]
        M303 = "m303" # 0x04BC dio_io[103]
        M304 = "m304" # 0x04C0 dio_io[104]
        M305 = "m305" # 0x04C4 dio_io[105]
        M306 = "m306" # 0x04C8 dio_io[106]
        M307 = "m307" # 0x04CC dio_io[107]
        M308 = "m308" # 0x04D0 dio_io[108]
        M309 = "m309" # 0x04D4 dio_io[109]
        M310 = "m310" # 0x04D8 dio_io[110]
        M311 = "m311" # 0x04DC dio_io[111]
        M312 = "m312" # 0x04E0 dio_io[112]
        M313 = "m313" # 0x04E4 dio_io[113]
        M314 = "m314" # 0x04E8 dio_io[114]
        M315 = "m315" # 0x04EC dio_io[115]
        M316 = "m316" # 0x04F0 dio_io[116]
        M317 = "m317" # 0x04F4 dio_io[117]
        M318 = "m318" # 0x04F8 dio_io[118]
        M319 = "m319" # 0x04FC dio_io[119]
        M320 = "m320" # 0x0500 dio_io[120]
        # Ethercat joiont information
        S000 = "s000" # 0x0500 cia402ctrl[0-5]
        S001 = "s001" # 0x0502 ctrl[0-5]
        S002 = "s002" # 0x0504 cia402targetpls[0-5]
        S003 = "s003" # 0x0508 notification[0-5]
        S004 = "s004" # 0x050C cia402sts[0-5]
        S005 = "s005" # 0x050E sts[0-5]
        S006 = "s006" # 0x0510 rtn[0-5]
        S007 = "s007" # 0x0512 cia402err[0-5]
        S008 = "s008" # 0x0514 alarm[0-5]
        S009 = "s009" # 0x0518 targetplsfb[0-5]
        S010 = "s010" # 0x051C cia402actualpls[0-5]
        S011 = "s011" # 0x0520 cia402followingerr[0-5]
        S012 = "s012" # 0x0524 observer_output_value[0-5]
        S013 = "s013" # 0x0528 torque[0-5]
        S014 = "s014" # 0x052A thermal[0-5]
        S015 = "s015" # 0x052C disturbance[0-5]
        S016 = "s016" # 0x052E gainrate[0-5]
        S017 = "s017" # 0x0530 polerate[0-5]
        S018 = "s018" # 0x0532 filtered_torque[0-5]
        S019 = "s019" # 0x0534 filtered_velocity[0-5]
        S020 = "s020" # 0x0536 filtered_D[0-5]
        S020 = "s020" # 0x0538 filtered_Q[0-5]
        # Force torque sensor information
        F000 = "f000" # 0x0700 sts
        F001 = "f001" # 0x0701 gain_sts
        F100 = "f100" # 0x0710 zero_point[0-7]
        F200 = "f200" # 0x0720 raw_value[0-7]
        F300 = "f300" # 0x0730 gain[0-7]
        # System management block information
        Y000 = "y000" # 0x0800 robtask_name[0-31]
        Y001 = "y001" # 0x0820 running_name[0-31]
        Y002 = "y002" # 0x0840 running_pid
        Y003 = "y003" # 0x0844 assign_port[0]
        Y004 = "y004" # 0x0846 assign_port[1]
        Y005 = "y005" # 0x0848 assign_port[2]
        Y006 = "y006" # 0x084A assign_port[3]
        Y007 = "y007" # 0x084C assign_port[4]
        Y008 = "y008" # 0x085E assign_port[5]
        Y009 = "y009" # 0x0850 assign_port[6]
        Y010 = "y010" # 0x0852 assign_port[7]
        Y011 = "y011" # 0x0854 assign_port[8]
        Y012 = "y012" # 0x0856 assign_port[9]
        Y013 = "y013" # 0x0858 assign_port[10]
        Y014 = "y014" # 0x085A assign_port[11]
        # User block information
        U000 = "u000" # 0x1800 intval[0]
        U001 = "u001" # 0x1804 intval[1]
        U002 = "u002" # 0x1808 intval[2]
        U003 = "u003" # 0x180C intval[3]
        U004 = "u004" # 0x1810 intval[4]
        U005 = "u005" # 0x1814 intval[5]
        U006 = "u006" # 0x1818 intval[6]
        U007 = "u007" # 0x181C intval[7]
        U008 = "u008" # 0x1820 intval[8]
        U009 = "u009" # 0x1824 intval[9]
        U010 = "u010" # 0x1828 intval[10]
        U011 = "u011" # 0x182C intval[11]
        U012 = "u012" # 0x1830 intval[12]
        U013 = "u013" # 0x1834 intval[13]
        U014 = "u014" # 0x1838 intval[14]
        U015 = "u015" # 0x183C intval[15]
        U016 = "u016" # 0x1840 intval[16]
        U017 = "u017" # 0x1844 intval[17]
        U018 = "u018" # 0x1848 intval[18]
        U019 = "u019" # 0x184C intval[19]
        U020 = "u020" # 0x1850 intval[20]
        U021 = "u021" # 0x1854 intval[21]
        U022 = "u022" # 0x1858 intval[22]
        U023 = "u023" # 0x185C intval[23]
        U024 = "u024" # 0x1860 intval[24]
        U025 = "u025" # 0x1864 intval[25]
        U026 = "u026" # 0x1868 intval[26]
        U027 = "u027" # 0x186C intval[27]
        U028 = "u028" # 0x1870 intval[28]
        U029 = "u029" # 0x1874 intval[29]
        U030 = "u030" # 0x1878 intval[30]
        U031 = "u031" # 0x187C intval[31]
        U032 = "u032" # 0x1880 intval[32]
        U033 = "u033" # 0x1884 intval[33]
        U034 = "u034" # 0x1888 intval[34]
        U035 = "u035" # 0x188C intval[35]
        U036 = "u036" # 0x1890 intval[36]
        U037 = "u037" # 0x1894 intval[37]
        U038 = "u038" # 0x1898 intval[38]
        U039 = "u039" # 0x189C intval[39]
        U040 = "u040" # 0x18A0 intval[40]
        U041 = "u041" # 0x18A4 intval[41]
        U042 = "u042" # 0x18A8 intval[42]
        U043 = "u043" # 0x18AC intval[43]
        U044 = "u044" # 0x18B0 intval[44]
        U045 = "u045" # 0x18B4 intval[45]
        U046 = "u046" # 0x18B8 intval[46]
        U047 = "u047" # 0x18BC intval[47]
        U048 = "u048" # 0x18C0 intval[48]
        U049 = "u049" # 0x18C4 intval[49]
        U050 = "u050" # 0x18C8 intval[50]
        U051 = "u051" # 0x18CC intval[51]
        U052 = "u052" # 0x18D0 intval[52]
        U053 = "u053" # 0x18D4 intval[53]
        U054 = "u054" # 0x18D8 intval[54]
        U055 = "u055" # 0x18DC intval[55]
        U056 = "u056" # 0x18E0 intval[56]
        U057 = "u057" # 0x18E4 intval[57]
        U058 = "u058" # 0x18E8 intval[58]
        U059 = "u059" # 0x18EC intval[59]
        U060 = "u060" # 0x18F0 intval[60]
        U061 = "u061" # 0x18F4 intval[61]
        U062 = "u062" # 0x18F8 intval[62]
        U063 = "u063" # 0x18FC intval[63]
        U064 = "u064" # 0x1900 intval[64]
        U065 = "u065" # 0x1904 intval[65]
        U066 = "u066" # 0x1908 intval[66]
        U067 = "u067" # 0x190C intval[67]
        U068 = "u068" # 0x1910 intval[68]
        U069 = "u069" # 0x1914 intval[69]
        U070 = "u070" # 0x1918 intval[70]
        U071 = "u071" # 0x191C intval[71]
        U072 = "u072" # 0x1920 intval[72]
        U073 = "u073" # 0x1924 intval[73]
        U074 = "u074" # 0x1928 intval[74]
        U075 = "u075" # 0x192C intval[75]
        U076 = "u076" # 0x1930 intval[76]
        U077 = "u077" # 0x1934 intval[77]
        U078 = "u078" # 0x1938 intval[78]
        U079 = "u079" # 0x193C intval[79]
        U080 = "u080" # 0x1940 intval[80]
        U081 = "u081" # 0x1944 intval[81]
        U082 = "u082" # 0x1948 intval[82]
        U083 = "u083" # 0x194C intval[83]
        U084 = "u084" # 0x1950 intval[84]
        U085 = "u085" # 0x1954 intval[85]
        U086 = "u086" # 0x1958 intval[86]
        U087 = "u087" # 0x195C intval[87]
        U088 = "u088" # 0x1960 intval[88]
        U089 = "u089" # 0x1964 intval[89]
        U090 = "u090" # 0x1968 intval[90]
        U091 = "u091" # 0x196C intval[91]
        U092 = "u092" # 0x1970 intval[92]
        U093 = "u093" # 0x1974 intval[93]
        U094 = "u094" # 0x1978 intval[94]
        U095 = "u095" # 0x197C intval[95]
        U096 = "u096" # 0x1980 intval[96]
        U097 = "u097" # 0x1984 intval[97]
        U098 = "u098" # 0x1988 intval[98]
        U099 = "u099" # 0x198C intval[99]
        U100 = "u100" # 0x1990 intval[100]
        U101 = "u101" # 0x1994 intval[101]
        U102 = "u102" # 0x1998 intval[102]
        U103 = "u103" # 0x199C intval[103]
        U104 = "u104" # 0x19A0 intval[104]
        U105 = "u105" # 0x19A4 intval[105]
        U106 = "u106" # 0x19A8 intval[106]
        U107 = "u107" # 0x19AC intval[107]
        U108 = "u108" # 0x19B0 intval[108]
        U109 = "u109" # 0x19B4 intval[109]
        U110 = "u110" # 0x19B8 intval[110]
        U111 = "u111" # 0x19BC intval[111]
        U112 = "u112" # 0x19C0 intval[112]
        U113 = "u113" # 0x19C4 intval[113]
        U114 = "u114" # 0x19C8 intval[114]
        U115 = "u115" # 0x19CC intval[115]
        U116 = "u116" # 0x19D0 intval[116]
        U117 = "u117" # 0x19D4 intval[117]
        U118 = "u118" # 0x19D8 intval[118]
        U119 = "u119" # 0x19DC intval[119]
        U120 = "u120" # 0x19E0 intval[120]
        U121 = "u121" # 0x19E4 intval[121]
        U122 = "u122" # 0x19E8 intval[122]
        U123 = "u123" # 0x19EC intval[123]
        U124 = "u124" # 0x19F0 intval[124]
        U125 = "u125" # 0x19F4 intval[125]
        U126 = "u126" # 0x19F8 intval[126]
        U127 = "u127" # 0x19FC intval[127]
        U128 = "u128" # 0x1A00 intval[128]
        U129 = "u129" # 0x1A04 intval[129]
        U130 = "u130" # 0x1A08 intval[130]
        U131 = "u131" # 0x1A0C intval[131]
        U132 = "u132" # 0x1A10 intval[132]
        U133 = "u133" # 0x1A14 intval[133]
        U134 = "u134" # 0x1A18 intval[134]
        U135 = "u135" # 0x1A1C intval[135]
        U136 = "u136" # 0x1A20 intval[136]
        U137 = "u137" # 0x1A24 intval[137]
        U138 = "u138" # 0x1A28 intval[138]
        U139 = "u139" # 0x1A2C intval[139]
        U140 = "u140" # 0x1A30 intval[140]
        U141 = "u141" # 0x1A34 intval[141]
        U142 = "u142" # 0x1A38 intval[142]
        U143 = "u143" # 0x1A3C intval[143]
        U144 = "u144" # 0x1A40 intval[144]
        U145 = "u145" # 0x1A44 intval[145]
        U146 = "u146" # 0x1A48 intval[146]
        U147 = "u147" # 0x1A4C intval[147]
        U148 = "u148" # 0x1A50 intval[148]
        U149 = "u149" # 0x1A54 intval[149]
        U150 = "u150" # 0x1A58 intval[150]
        U151 = "u151" # 0x1A5C intval[151]
        U152 = "u152" # 0x1A60 intval[152]
        U153 = "u153" # 0x1A64 intval[153]
        U154 = "u154" # 0x1A68 intval[154]
        U155 = "u155" # 0x1A6C intval[155]
        U156 = "u156" # 0x1A70 intval[156]
        U157 = "u157" # 0x1A74 intval[157]
        U158 = "u158" # 0x1A78 intval[158]
        U159 = "u159" # 0x1A7C intval[159]
        U160 = "u160" # 0x1A80 intval[160]
        U161 = "u161" # 0x1A84 intval[161]
        U162 = "u162" # 0x1A88 intval[162]
        U163 = "u163" # 0x1A8C intval[163]
        U164 = "u164" # 0x1A90 intval[164]
        U165 = "u165" # 0x1A94 intval[165]
        U166 = "u166" # 0x1A98 intval[166]
        U167 = "u167" # 0x1A9C intval[167]
        U168 = "u168" # 0x1AA0 intval[168]
        U169 = "u169" # 0x1AA4 intval[169]
        U170 = "u170" # 0x1AA8 intval[170]
        U171 = "u171" # 0x1AAC intval[171]
        U172 = "u172" # 0x1AB0 intval[172]
        U173 = "u173" # 0x1AB4 intval[173]
        U174 = "u174" # 0x1AB8 intval[174]
        U175 = "u175" # 0x1ABC intval[175]
        U176 = "u176" # 0x1AC0 intval[176]
        U177 = "u177" # 0x1AC4 intval[177]
        U178 = "u178" # 0x1AC8 intval[178]
        U179 = "u179" # 0x1ACC intval[179]
        U180 = "u180" # 0x1AD0 intval[180]
        U181 = "u181" # 0x1AD4 intval[181]
        U182 = "u182" # 0x1AD8 intval[182]
        U183 = "u183" # 0x1ADC intval[183]
        U184 = "u184" # 0x1AE0 intval[184]
        U185 = "u185" # 0x1AE4 intval[185]
        U186 = "u186" # 0x1AE8 intval[186]
        U187 = "u187" # 0x1AEC intval[187]
        U188 = "u188" # 0x1AF0 intval[188]
        U189 = "u189" # 0x1AF4 intval[189]
        U190 = "u190" # 0x1AF8 intval[190]
        U191 = "u191" # 0x1AFC intval[191]
        U192 = "u192" # 0x1B00 intval[192]
        U193 = "u193" # 0x1B04 intval[193]
        U194 = "u194" # 0x1B08 intval[194]
        U195 = "u195" # 0x1B0C intval[195]
        U196 = "u196" # 0x1B10 intval[196]
        U197 = "u197" # 0x1B14 intval[197]
        U198 = "u198" # 0x1B18 intval[198]
        U199 = "u199" # 0x1B1C intval[199]
        U200 = "u200" # 0x1B20 intval[200]
        U201 = "u201" # 0x1B24 intval[201]
        U202 = "u202" # 0x1B28 intval[202]
        U203 = "u203" # 0x1B2C intval[203]
        U204 = "u204" # 0x1B30 intval[204]
        U205 = "u205" # 0x1B34 intval[205]
        U206 = "u206" # 0x1B38 intval[206]
        U207 = "u207" # 0x1B3C intval[207]
        U208 = "u208" # 0x1B40 intval[208]
        U209 = "u209" # 0x1B44 intval[209]
        U210 = "u210" # 0x1B48 intval[210]
        U211 = "u211" # 0x1B4C intval[211]
        U212 = "u212" # 0x1B50 intval[212]
        U213 = "u213" # 0x1B54 intval[213]
        U214 = "u214" # 0x1B58 intval[214]
        U215 = "u215" # 0x1B5C intval[215]
        U216 = "u216" # 0x1B60 intval[216]
        U217 = "u217" # 0x1B64 intval[217]
        U218 = "u218" # 0x1B68 intval[218]
        U219 = "u219" # 0x1B6C intval[219]
        U220 = "u220" # 0x1B70 intval[220]
        U221 = "u221" # 0x1B74 intval[221]
        U222 = "u222" # 0x1B78 intval[222]
        U223 = "u223" # 0x1B7C intval[223]
        U224 = "u224" # 0x1B80 intval[224]
        U225 = "u225" # 0x1B84 intval[225]
        U226 = "u226" # 0x1B88 intval[226]
        U227 = "u227" # 0x1B8C intval[227]
        U228 = "u228" # 0x1B90 intval[228]
        U229 = "u229" # 0x1B94 intval[229]
        U230 = "u230" # 0x1B98 intval[230]
        U231 = "u231" # 0x1B9C intval[231]
        U232 = "u232" # 0x1BA0 intval[232]
        U233 = "u233" # 0x1BA4 intval[233]
        U234 = "u234" # 0x1BA8 intval[234]
        U235 = "u235" # 0x1BAC intval[235]
        U236 = "u236" # 0x1BB0 intval[236]
        U237 = "u237" # 0x1BB4 intval[237]
        U238 = "u238" # 0x1BB8 intval[238]
        U239 = "u239" # 0x1BBC intval[239]
        U240 = "u240" # 0x1BC0 intval[240]
        U241 = "u241" # 0x1BC4 intval[241]
        U242 = "u242" # 0x1BC8 intval[242]
        U243 = "u243" # 0x1BCC intval[243]
        U244 = "u244" # 0x1BD0 intval[244]
        U245 = "u245" # 0x1BD4 intval[245]
        U246 = "u246" # 0x1BD8 intval[246]
        U247 = "u247" # 0x1BDC intval[247]
        U248 = "u248" # 0x1BE0 intval[248]
        U249 = "u249" # 0x1BE4 intval[249]
        U250 = "u250" # 0x1BE8 intval[250]
        U251 = "u251" # 0x1BEC intval[251]
        U252 = "u252" # 0x1BF0 intval[252]
        U253 = "u253" # 0x1BF4 intval[253]
        U254 = "u254" # 0x1BF8 intval[254]
        U255 = "u255" # 0x1BFC intval[255]
        M300 = "m300" # 0x1C00 floatval[0]
        M301 = "m301" # 0x1C08 floatval[1]
        M302 = "m302" # 0x1C10 floatval[2]
        M303 = "m303" # 0x1C18 floatval[3]
        M304 = "m304" # 0x1C20 floatval[4]
        M305 = "m305" # 0x1C28 floatval[5]
        M306 = "m306" # 0x1C30 floatval[6]
        M307 = "m307" # 0x1C38 floatval[7]
        M308 = "m308" # 0x1C40 floatval[8]
        M309 = "m309" # 0x1C48 floatval[9]
        M310 = "m310" # 0x1C50 floatval[10]
        M311 = "m311" # 0x1C58 floatval[11]
        M312 = "m312" # 0x1C60 floatval[12]
        M313 = "m313" # 0x1C68 floatval[13]
        M314 = "m314" # 0x1C70 floatval[14]
        M315 = "m315" # 0x1C78 floatval[15]
        M316 = "m316" # 0x1C80 floatval[16]
        M317 = "m317" # 0x1C88 floatval[17]
        M318 = "m318" # 0x1C90 floatval[18]
        M319 = "m319" # 0x1C98 floatval[19]
        M320 = "m320" # 0x1CA0 floatval[20]
        M321 = "m321" # 0x1CA8 floatval[21]
        M322 = "m322" # 0x1CB0 floatval[22]
        M323 = "m323" # 0x1CB8 floatval[23]
        M324 = "m324" # 0x1CC0 floatval[24]
        M325 = "m325" # 0x1CC8 floatval[25]
        M326 = "m326" # 0x1CD0 floatval[26]
        M327 = "m327" # 0x1CD8 floatval[27]
        M328 = "m328" # 0x1CE0 floatval[28]
        M329 = "m329" # 0x1CE8 floatval[29]
        M330 = "m330" # 0x1CF0 floatval[30]
        M331 = "m331" # 0x1CF8 floatval[31]
        M332 = "m332" # 0x1D00 floatval[32]
        M333 = "m333" # 0x1D08 floatval[33]
        M334 = "m334" # 0x1D10 floatval[34]
        M335 = "m335" # 0x1D18 floatval[35]
        M336 = "m336" # 0x1D20 floatval[36]
        M337 = "m337" # 0x1D28 floatval[37]
        M338 = "m338" # 0x1D30 floatval[38]
        M339 = "m339" # 0x1D38 floatval[39]
        M340 = "m340" # 0x1D40 floatval[40]
        M341 = "m341" # 0x1D48 floatval[41]
        M342 = "m342" # 0x1D50 floatval[42]
        M343 = "m343" # 0x1D58 floatval[43]
        M344 = "m344" # 0x1D60 floatval[44]
        M345 = "m345" # 0x1D68 floatval[45]
        M346 = "m346" # 0x1D70 floatval[46]
        M347 = "m347" # 0x1D78 floatval[47]
        M348 = "m348" # 0x1D80 floatval[48]
        M349 = "m349" # 0x1D88 floatval[49]
        M350 = "m350" # 0x1D90 floatval[50]
        M351 = "m351" # 0x1D98 floatval[51]
        M352 = "m352" # 0x1DA0 floatval[52]
        M353 = "m353" # 0x1DA8 floatval[53]
        M354 = "m354" # 0x1DB0 floatval[54]
        M355 = "m355" # 0x1DB8 floatval[55]
        M356 = "m356" # 0x1DC0 floatval[56]
        M357 = "m357" # 0x1DC8 floatval[57]
        M358 = "m358" # 0x1DD0 floatval[58]
        M359 = "m359" # 0x1DD8 floatval[59]
        M360 = "m360" # 0x1DE0 floatval[60]
        M361 = "m361" # 0x1DE8 floatval[61]
        M362 = "m362" # 0x1DF0 floatval[62]
        M363 = "m363" # 0x1DF8 floatval[63]
        M364 = "m364" # 0x1E00 floatval[64]
        M365 = "m365" # 0x1E08 floatval[65]
        M366 = "m366" # 0x1E10 floatval[66]
        M367 = "m367" # 0x1E18 floatval[67]
        M368 = "m368" # 0x1E20 floatval[68]
        M369 = "m369" # 0x1E28 floatval[69]
        M370 = "m370" # 0x1E30 floatval[70]
        M371 = "m371" # 0x1E38 floatval[71]
        M372 = "m372" # 0x1E40 floatval[72]
        M373 = "m373" # 0x1E48 floatval[73]
        M374 = "m374" # 0x1E50 floatval[74]
        M375 = "m375" # 0x1E58 floatval[75]
        M376 = "m376" # 0x1E60 floatval[76]
        M377 = "m377" # 0x1E68 floatval[77]
        M378 = "m378" # 0x1E70 floatval[78]
        M379 = "m379" # 0x1E78 floatval[79]
        M380 = "m380" # 0x1E80 floatval[80]
        M381 = "m381" # 0x1E88 floatval[81]
        M382 = "m382" # 0x1E90 floatval[82]
        M383 = "m383" # 0x1E98 floatval[83]
        M384 = "m384" # 0x1EA0 floatval[84]
        M385 = "m385" # 0x1EA8 floatval[85]
        M386 = "m386" # 0x1EB0 floatval[86]
        M387 = "m387" # 0x1EB8 floatval[87]
        M388 = "m388" # 0x1EC0 floatval[88]
        M389 = "m389" # 0x1EC8 floatval[89]
        M390 = "m390" # 0x1ED0 floatval[90]
        M391 = "m391" # 0x1ED8 floatval[91]
        M392 = "m392" # 0x1EE0 floatval[92]
        M393 = "m393" # 0x1EE8 floatval[93]
        M394 = "m394" # 0x1EF0 floatval[94]
        M395 = "m395" # 0x1EF8 floatval[95]
        M396 = "m396" # 0x1F00 floatval[96]
        M397 = "m397" # 0x1F08 floatval[97]
        M398 = "m398" # 0x1F10 floatval[98]
        M399 = "m399" # 0x1F18 floatval[99]
        M400 = "m400" # 0x1F20 floatval[100]
        M401 = "m401" # 0x1F28 floatval[101]
        M402 = "m402" # 0x1F30 floatval[102]
        M403 = "m403" # 0x1F38 floatval[103]
        M404 = "m404" # 0x1F40 floatval[104]
        M405 = "m405" # 0x1F48 floatval[105]
        M406 = "m406" # 0x1F50 floatval[106]
        M407 = "m407" # 0x1F58 floatval[107]
        M408 = "m408" # 0x1F60 floatval[108]
        M409 = "m409" # 0x1F68 floatval[109]
        M410 = "m410" # 0x1F70 floatval[110]
        M411 = "m411" # 0x1F78 floatval[111]
        M412 = "m412" # 0x1F80 floatval[112]
        M413 = "m413" # 0x1F88 floatval[113]
        M414 = "m414" # 0x1F90 floatval[114]
        M415 = "m415" # 0x1F98 floatval[115]
        M416 = "m416" # 0x1FA0 floatval[116]
        M417 = "m417" # 0x1FA8 floatval[117]
        M418 = "m418" # 0x1FB0 floatval[118]
        M419 = "m419" # 0x1FB8 floatval[119]
        M420 = "m420" # 0x1FC0 floatval[120]
        M421 = "m421" # 0x1FC8 floatval[121]
        M422 = "m422" # 0x1FD0 floatval[122]
        M423 = "m423" # 0x1FD8 floatval[123]
        M424 = "m424" # 0x1FE0 floatval[124]
        M425 = "m425" # 0x1FE8 floatval[125]
        M426 = "m426" # 0x1FF0 floatval[126]
        M427 = "m427" # 0x1FF8 floatval[127]
        M428 = "m428" # 0x2000 floatval[128]
        M429 = "m429" # 0x2008 floatval[129]
        M430 = "m430" # 0x2010 floatval[130]
        M431 = "m431" # 0x2018 floatval[131]
        M432 = "m432" # 0x2020 floatval[132]
        M433 = "m433" # 0x2028 floatval[133]
        M434 = "m434" # 0x2030 floatval[134]
        M435 = "m435" # 0x2038 floatval[135]
        M436 = "m436" # 0x2040 floatval[136]
        M437 = "m437" # 0x2048 floatval[137]
        M438 = "m438" # 0x2050 floatval[138]
        M439 = "m439" # 0x2058 floatval[139]
        M440 = "m440" # 0x2060 floatval[140]
        M441 = "m441" # 0x2068 floatval[141]
        M442 = "m442" # 0x2070 floatval[142]
        M443 = "m443" # 0x2078 floatval[143]
        M444 = "m444" # 0x2080 floatval[144]
        M445 = "m445" # 0x2088 floatval[145]
        M446 = "m446" # 0x2090 floatval[146]
        M447 = "m447" # 0x2098 floatval[147]
        M448 = "m448" # 0x20A0 floatval[148]
        M449 = "m449" # 0x20A8 floatval[149]
        M450 = "m450" # 0x20B0 floatval[150]
        M451 = "m451" # 0x20B8 floatval[151]
        M452 = "m452" # 0x20C0 floatval[152]
        M453 = "m453" # 0x20C8 floatval[153]
        M454 = "m454" # 0x20D0 floatval[154]
        M455 = "m455" # 0x20D8 floatval[155]
        M456 = "m456" # 0x20E0 floatval[156]
        M457 = "m457" # 0x20E8 floatval[157]
        M458 = "m458" # 0x20F0 floatval[158]
        M459 = "m459" # 0x20F8 floatval[159]
        M460 = "m460" # 0x2100 floatval[160]
        M461 = "m461" # 0x2108 floatval[161]
        M462 = "m462" # 0x2110 floatval[162]
        M463 = "m463" # 0x2118 floatval[163]
        M464 = "m464" # 0x2120 floatval[164]
        M465 = "m465" # 0x2128 floatval[165]
        M466 = "m466" # 0x2130 floatval[166]
        M467 = "m467" # 0x2138 floatval[167]
        M468 = "m468" # 0x2140 floatval[168]
        M469 = "m469" # 0x2148 floatval[169]
        M470 = "m470" # 0x2150 floatval[170]
        M471 = "m471" # 0x2158 floatval[171]
        M472 = "m472" # 0x2160 floatval[172]
        M473 = "m473" # 0x2168 floatval[173]
        M474 = "m474" # 0x2170 floatval[174]
        M475 = "m475" # 0x2178 floatval[175]
        M476 = "m476" # 0x2180 floatval[176]
        M477 = "m477" # 0x2188 floatval[177]
        M478 = "m478" # 0x2190 floatval[178]
        M479 = "m479" # 0x2198 floatval[179]
        M480 = "m480" # 0x21A0 floatval[180]
        M481 = "m481" # 0x21A8 floatval[181]
        M482 = "m482" # 0x21B0 floatval[182]
        M483 = "m483" # 0x21B8 floatval[183]
        M484 = "m484" # 0x21C0 floatval[184
        M485 = "m485" # 0x21C8 floatval[185]
        M486 = "m486" # 0x21D0 floatval[186]
        M487 = "m487" # 0x21D8 floatval[187]
        M488 = "m488" # 0x21E0 floatval[188]
        M489 = "m489" # 0x21E8 floatval[189]
        M490 = "m490" # 0x21F0 floatval[190]
        M491 = "m491" # 0x21F8 floatval[191]
        M492 = "m492" # 0x2200 floatval[192]
        M493 = "m493" # 0x2208 floatval[193]
        M494 = "m494" # 0x2210 floatval[194]
        M495 = "m495" # 0x2218 floatval[195]
        M496 = "m496" # 0x2220 floatval[196]
        M497 = "m497" # 0x2228 floatval[197]
        M498 = "m498" # 0x2230 floatval[198]
        M499 = "m499" # 0x2238 floatval[199]
        M500 = "m500" # 0x2240 floatval[200]
        M501 = "m501" # 0x2248 floatval[201]
        M502 = "m502" # 0x2250 floatval[202]
        M503 = "m503" # 0x2258 floatval[203]
        M504 = "m504" # 0x2260 floatval[204]
        M505 = "m505" # 0x2268 floatval[205]
        M506 = "m506" # 0x2270 floatval[206]
        M507 = "m507" # 0x2278 floatval[207]
        M508 = "m508" # 0x2280 floatval[208]
        M509 = "m509" # 0x2288 floatval[209]
        M510 = "m510" # 0x2290 floatval[210]
        M511 = "m511" # 0x2298 floatval[211]
        M512 = "m512" # 0x22A0 floatval[212]
        M513 = "m513" # 0x22A8 floatval[213]
        M514 = "m514" # 0x22B0 floatval[214]
        M515 = "m515" # 0x22B8 floatval[215]
        M516 = "m516" # 0x22C0 floatval[216]
        M517 = "m517" # 0x22C8 floatval[217]
        M518 = "m518" # 0x22D0 floatval[218]
        M519 = "m519" # 0x22D8 floatval[219]
        M520 = "m520" # 0x22E0 floatval[220]
        M521 = "m521" # 0x22E8 floatval[221]
        M522 = "m522" # 0x22F0 floatval[222]
        M523 = "m523" # 0x22F8 floatval[223]
        M524 = "m524" # 0x2300 floatval[224]
        M525 = "m525" # 0x2308 floatval[225]
        M526 = "m526" # 0x2310 floatval[226]
        M527 = "m527" # 0x2318 floatval[227]
        M528 = "m528" # 0x2320 floatval[228]
        M529 = "m529" # 0x2328 floatval[229]
        M530 = "m530" # 0x2330 floatval[230]
        M531 = "m531" # 0x2338 floatval[231]
        M532 = "m532" # 0x2340 floatval[232]
        M533 = "m533" # 0x2348 floatval[233]
        M534 = "m534" # 0x2350 floatval[234]
        M535 = "m535" # 0x2358 floatval[235]
        M536 = "m536" # 0x2360 floatval[236]
        M537 = "m537" # 0x2368 floatval[237]
        M538 = "m538" # 0x2370 floatval[238]
        M539 = "m539" # 0x2378 floatval[239]
        M540 = "m540" # 0x2380 floatval[240]
        M541 = "m541" # 0x2388 floatval[241]
        M542 = "m542" # 0x2390 floatval[242]
        M543 = "m543" # 0x2398 floatval[243]
        M544 = "m544" # 0x23A0 floatval[244]
        M545 = "m545" # 0x23A8 floatval[245]
        M546 = "m546" # 0x23B0 floatval[246]
        M547 = "m547" # 0x23B8 floatval[247]
        M548 = "m548" # 0x23C0 floatval[248]
        M549 = "m549" # 0x23C8 floatval[249]
        M550 = "m550" # 0x23D0 floatval[250]
        M551 = "m551" # 0x23D8 floatval[251]
        M552 = "m552" # 0x23E0 floatval[252]
        M553 = "m553" # 0x23E8 floatval[253]
        M554 = "m554" # 0x23F0 floatval[254]
        M555 = "m555" # 0x23F8 floatval[255]
        # Controller state information
        C000 = "c000" # 0x2800 errcode
        C001 = "c001" # 0x2802 bTeachMode
        C002 = "c002" # 0x2804 bSPILargeFrame
        # Robot configuration information
        G000 = "g000" # 0x2C00 manip_type[0-35]
        G001 = "g001" # 0x2C24 manip_serial[0-35]
        G002 = "g002" # 0x2C48 format_version[0-2]
        G003 = "g003" # 0x2C54 parameter_version[0-2]
        # Robot status information
        R000 = "r000" # 0x3000 cmdx,cmdy,cmdz,cmdrz,cmdry,cmdrx
        R100 = "r100" # 0x3040 posture
        R101 = "r101" # 0x3044 coordinate
        R102 = "r102" # 0x3048 singular
        R103 = "r103" # 0x304C multiturn
        R200 = "r200" # 0x3050 joint[0-5]
        R300 = "r300" # 0x3090 velocity
        R301 = "r301" # 0x3098 vel_error_axes
        R302 = "r302" # 0x309C softlimit
        R303 = "r303" # 0x30A0 joint_svon_to_svoff[0-5]
        R304 = "r304" # 0x30E0 b_saved
        R305 = "r305" # 0x30E4 toolno
        R306 = "r306" # 0x30E8 hdorgx,hdorgy,hdorgz,hdorgrz,hdorgry,hdorgrx
        R400 = "r400" # 0x3128 carte_svon_to_svoff[0-5]
        R401 = "r401" # 0x3168 svon_to_svoff_posture
        R402 = "r402" # 0x316C svon_to_svoff_coordinate
        R403 = "r403" # 0x3170 svon_to_svoff_singular
        R404 = "r404" # 0x3174 svon_to_svoff_multiturn
        R405 = "r405" # 0x3178 svon_to_svoff_toolno
        R406 = "r406" # 0x317C bRequestHold
        R407 = "r407" # 0x317E bRequestSuspend
        R408 = "r408" # 0x3180 bSuspended
        R409 = "r409" # 0x3184 permitted_worker_id
        R410 = "r410" # 0x3188 tool_org_params[0-5]
        R411 = "r411" # 0x31B8 tool_fwdmatrix[0-11]
        R412 = "r412" # 0x3218 last_hold_factor
        R413 = "r413" # 0x3219 vdesc0_sts
        R414 = "r414" # 0x321A vdesc1_sts
        R415 = "r415" # 0x321B n_queued
        R416 = "r416" # 0x321C logical_cmd_pulse[0-5]
        R417 = "r417" # 0x323C logical_fb_pulse[0-5]
        R418 = "r418" # 0x325C holdinfo
        R419 = "r419" # 0x3260 svsts
        R419 = "r419" # 0x3264 manip_pwr
        R420 = "r420" # 0x3266 ems
        R421 = "r421" # 0x3268 vdesc0_mvid
        R422 = "r422" # 0x326C vdesc1_mvid

if __name__ == '__main__':
    c = InfoCatchClient()
    def callback(data):
        print(data)
    c.connect([InfoCatchClient.Label.I000,
                InfoCatchClient.Label.R200,
                InfoCatchClient.Label.M000,
                InfoCatchClient.Label.M001,
                InfoCatchClient.Label.M100,
                InfoCatchClient.Label.M102,
                InfoCatchClient.Label.F000,
                InfoCatchClient.Label.F200,
                InfoCatchClient.Label.F300,
                ])
    for x in range(10):
        data = c.recv()
        print(data)
    c.close()


