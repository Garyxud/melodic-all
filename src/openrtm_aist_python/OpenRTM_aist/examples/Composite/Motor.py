#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

motor_spec = ["implementation_id", "Motor", 
              "type_name",         "Motor", 
              "description",       "Motor component", 
              "version",           "1.0", 
              "vendor",            "Noriaki Ando, AIST", 
              "category",          "example", 
              "activity_type",     "DataFlowComponent", 
              "max_instance",      "10", 
              "language",          "Python", 
              "lang_type",         "SCRIPT",
              "conf.default.motor_id", "0",
              ""]

class Motor(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return

  def onInitialize(self):
    self._d_in = RTC.TimedFloat(RTC.Time(0,0),0)
    self._inIn = OpenRTM_aist.InPort("in", self._d_in)
    self._d_out = RTC.TimedLong(RTC.Time(0,0),0)
    self._outOut = OpenRTM_aist.OutPort("out", self._d_out)

    # Set InPort buffers
    self.addInPort("in",self._inIn)
    
    # Set OutPort buffers
    self.addOutPort("out",self._outOut)

    self._motor_id = [0]
    
    # Bind variables and configuration variable
    self.bindParameter("motor_id", self._motor_id, "0")
    self._configsets.update("default")
    return RTC.RTC_OK


  def onExecute(self, ec_id):
    if self._inIn.isNew():
      data = self._inIn.read()
      print "Motor Received data: ", data.data
      self._d_out.data = long(data.data *2)
      self._outOut.write()
    return RTC.RTC_OK
  



def MotorInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=motor_spec)
  manager.registerFactory(profile,
                          Motor,
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  MotorInit(manager)

  # Create a component
  comp = manager.createComponent("Motor")



def main():
  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager()

if __name__ == "__main__":
  main()

