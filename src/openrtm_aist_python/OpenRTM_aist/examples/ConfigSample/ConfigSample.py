#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import time

import RTC
import OpenRTM_aist

# Module specification
configsample_spec = ["implementation_id", "ConfigSample",
                     "type_name",         "ConfigSample",
                     "description",       "Configuration example component",
                     "version",           "1.0",
                     "vendor",            "Shinji Kurihara",
                     "category",          "example",
                     "activity_type",     "DataFlowComponent",
                     "max_instance",      "10",
                     "language",          "Python",
                     "lang_type",         "script",
                     "conf.default.int_param0", "0",
                     "conf.default.int_param1", "1",
                     "conf.default.double_param0", "0.11",
                     "conf.default.double_param1", "9.9",
                     "conf.default.str_param0", "hoge",
                     "conf.default.str_param1", "dara",
                     "conf.default.vector_param0", "0.0,1.0,2.0,3.0,4.0",
                     ""]

i = 0
maxlen = 0

def ticktack():
  global i
  str_ = "/-\\|"
  i = (i+1) % 4
  return str_[i]


class ConfigSample(OpenRTM_aist.DataFlowComponentBase):
  # class constructor
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    print "ConfigSample constructor."
    self._int_param0 = [0]
    self._int_param1 = [1]
    self._double_param0 = [0.11]
    self._double_param1 = [9.9]
    self._str_param0 = ["hoge"]
    self._str_param1 = ["dara"]
    self._vector_param0 = [[0.0, 1.0, 2.0, 3.0, 4.0]]
    
  # The initialize action (on CREATED->ALIVE transition)
  def onInitialize(self):
    self.bindParameter("int_param0", self._int_param0, "0")
    self.bindParameter("int_param1", self._int_param1, "1")
    self.bindParameter("double_param0", self._double_param0, "0.11")
    self.bindParameter("double_param1", self._double_param1, "9.9")
    self.bindParameter("str_param0", self._str_param0, "hoge")
    self.bindParameter("str_param1", self._str_param1, "dara")
    self.bindParameter("vector_param0", self._vector_param0, "0.0,1.0,2.0,3.0,4.0")
  

    print "\n Please change configuration values from RtcLink"
    
    return RTC.RTC_OK

  # The execution action that is invoked periodically
  def onExecute(self, ec_id):
    global maxlen
    curlen = 0
    c = "                    "

    print "---------------------------------------"
    print " Active Configuration Set: ", self._configsets.getActiveId(),c
    print "---------------------------------------"
    
    print "int_param0:       ", self._int_param0, c
    print "int_param1:       ", self._int_param1, c
    print "double_param0:    ", self._double_param0, c
    print "double_param1:    ", self._double_param1, c
    print "str_param0:       ", self._str_param0, c
    print "str_param1:       ", self._str_param1, c

    for idx in range(len(self._vector_param0[0])):
      print "vector_param0[", idx, "]: ", self._vector_param0[0][idx], c

    print "---------------------------------------"

    curlen = len(self._vector_param0[0])

    if maxlen > curlen:
      maxlen = maxlen
    else:
      maxlen = curlen
        
    for idx in range(maxlen - curlen):
      print c, c

    print "Updating.... ", ticktack(), c

    str_ = ""
    for idx in range(12 + maxlen):
      str_ += "[A\r"
    print str_


    if self._int_param0 > 1000 and self._int_param0 < 1000000:
      time.sleep(self._int_param0/1000000.0)
    else:
      time.sleep(0.1)

    return RTC.RTC_OK


def ConfigSampleInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=configsample_spec)
  manager.registerFactory(profile,
                          ConfigSample,
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  ConfigSampleInit(manager)

  # Create a component
  comp = manager.createComponent("ConfigSample")

  # Activate component
  poa = manager.getPOA()
  obj = comp._default_POA().servant_to_reference(comp)
  rtobj = obj._narrow(RTC.RTObject)

  ecs = rtobj.get_owned_contexts()
  ecs[0].activate_component(rtobj)
    

def main():
  # Initialize manager
  mgr = OpenRTM_aist.Manager.init(sys.argv)

  # Set module initialization proceduer
  # This procedure will be invoked in activateManager() function.
  mgr.setModuleInitProc(MyModuleInit)

  # Activate manager and register to naming service
  mgr.activateManager()

  # run the manager in blocking mode
  # runManager(False) is the default
  mgr.runManager()

  # If you want to run the manager in non-blocking mode, do like this
  # mgr.runManager(True)

if __name__ == "__main__":
  main()
