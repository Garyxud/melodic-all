#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import time

import RTC
import OpenRTM_aist

consoleout_spec = ["implementation_id", "ConsoleOut",
                   "type_name",         "ConsoleOut",
                   "description",       "Console output component",
                   "version",           "1.0",
                   "vendor",            "Shinji Kurihara",
                   "category",          "example",
                   "activity_type",     "DataFlowComponent",
                   "max_instance",      "10",
                   "language",          "Python",
                   "lang_type",         "script",
                   ""]


class ConsoleOut(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return

  def onInitialize(self):
    self._data = RTC.TimedLong(RTC.Time(0,0),0)
    self._inport = OpenRTM_aist.InPort("in", self._data)
    # Set InPort buffer
    self.addInPort("in", self._inport)
    return RTC.RTC_OK

  def onExecute(self, ec_id):
    if self._inport.isNew():
      data = self._inport.read()
      print "Received: ", data.data
      print "TimeStamp: ", data.tm.sec, "[s] ", data.tm.nsec, "[ns]"
    time.sleep(0.001)
    return RTC.RTC_OK


def MyModuleInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=consoleout_spec)
  manager.registerFactory(profile,
                          ConsoleOut,
                          OpenRTM_aist.Delete)

  # Create a component
  comp = manager.createComponent("ConsoleOut")


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
