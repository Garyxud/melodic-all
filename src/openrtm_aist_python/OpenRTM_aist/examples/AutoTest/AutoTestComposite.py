#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import time
import socket

from rtc_handle10_11 import *
from CorbaNaming import *
import RTM



env = RtmEnv(sys.argv, ["localhost:2809"])

## Get Manager object reference
mgr_name = socket.gethostname()+".host_cxt/manager.mgr"
naming = CorbaNaming(env.orb, "localhost:2809")
manager = naming.resolve(mgr_name)._narrow(RTM.Manager)

listo = env.name_space["localhost:2809"].list_obj()
env.name_space['localhost:2809'].rtc_handles.keys()

ns = env.name_space['localhost:2809']

comp = ns.rtc_handles["PeriodicECSharedComposite0.rtc"]
config = comp.rtc_ref.get_configuration()
configset = config.get_configuration_sets()
config_data = configset[0].configuration_data


time.sleep(1)


motor = ns.rtc_handles["Motor0.rtc"]
sensor = ns.rtc_handles["Sensor0.rtc"]
controller = ns.rtc_handles["Controller0.rtc"]
loop_count = 1000
for i in range(loop_count):
  manager.create_component("PeriodicECSharedComposite?instance_name=aaa")
  env.name_space["localhost:2809"].list_obj()
  aaa=ns.rtc_handles[socket.gethostname()+".host_cxt/aaa.rtc"]
  org=aaa.rtc_ref.get_owned_organizations()[0]
  org.set_members ([motor.rtc_ref,sensor.rtc_ref,controller.rtc_ref])
  time.sleep(1)
  ret = org.remove_member("Motor0")
  ret = org.remove_member("Sensor0")
  ret = org.remove_member("Controller0")
  aaa.rtc_ref.exit()
  time.sleep(1)
