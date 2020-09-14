#!/usr/bin/env python

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
# --------

import sys
import rospy
from fsrobo_r_msgs.srv import *
from fsrobo_r_msgs.msg import *
from threading import Event

class IOFunctionType:
    SET_DIGITAL_OUT = 1
    SET_ADC_MODE = 2

class IOState:
    NO_CHANGE = -1
    ON = 1
    OFF = 0

class IOInterface:
    digital_states = {}
    analog_states = {}
  
    def __init__(self, robot_name=None, init_node=True):
        if robot_name == '':
            self.ns = '/'
        elif robot_name:
            self.ns = '/' + robot_name + '/'
        else:
            self.ns = ''

        self._init_node = init_node
        self.init_setter()
        self.init_getter()
  
    def get_digital_val(self, addr):
        return self.digital_states.get(addr, False)

    def get_digital_vals(self):
        return self.digital_states

    def get_analog_val(self, addr):
        return self.analog_states.get(addr, 0)

    def wait_digital_val(self, addr, val, time_out=None):
        ev = Event()

        if isinstance(val, list):
            val_list = val
        else:
            val_list = [val]

        def checker():
            addr_list = range(addr, addr + len(val_list))

            if all(map(lambda a, v: v not in [0, 1] or self.get_digital_val(a) == v, addr_list, val_list)):
                ev.set()

        self.digital_val_hooks.append(checker)
        result = ev.wait(time_out)
        self.digital_val_hooks.remove(checker)

        return result
  
    def set_digital_val(self, addr, *val):
        try:
            self.set_io(IOFunctionType.SET_DIGITAL_OUT, addr, list(val))
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
  
    def set_analog_mode(self, ch, mode):
        try:
            self.set_io(IOFunctionType.SET_ADC_MODE, ch, [mode])
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
  
    def callback(self, data):
        for x in data.digital_in_states:
            self.digital_states[x.addr] = x.state

        for x in data.digital_out_states:
            self.digital_states[x.addr] = x.state
        
        for x in data.analog_in_states:
            self.analog_states[x.ch] = x.state
        
        for hook in self.digital_val_hooks:
            hook()
        
    def init_getter(self):
        self.digital_val_hooks = []
        if self._init_node:
            rospy.init_node('io_interface', anonymous = True)
        rospy.Subscriber(self.ns + 'io_states', IOStates, self.callback)
        rospy.wait_for_message(self.ns + 'io_states', IOStates, 5)
  
    def init_setter(self):
        rospy.loginfo(self.ns)
        rospy.wait_for_service(self.ns + 'set_io')
        self.set_io = rospy.ServiceProxy(self.ns + 'set_io', SetIO)
  
if __name__ == '__main__':
    io = IOInterface()
    io.set_analog_mode(0,1)
    io.set_digital_val(16,1)
    io.set_digital_val(17,0)
    print(io.wait_digital_val(16, [1, 0], time_out=5))
    print io.digital_states
    print io.get_digital_val(0)
    print io.get_digital_val(16)
    print io.get_digital_val(17)
  
