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


class DataListener(OpenRTM_aist.ConnectorDataListenerT):
  def __init__(self, name):
    self._name = name

  def __del__(self):
    print "dtor of ", self._name

  def __call__(self, info, cdrdata):
    data = OpenRTM_aist.ConnectorDataListenerT.__call__(self, info, cdrdata, RTC.TimedLong(RTC.Time(0,0),0))
    print "------------------------------"
    print "Listener:       ", self._name
    print "Profile::name:  ", info.name
    print "Profile::id:    ", info.id
    print "Data:           ", data.data
    print "------------------------------"


class ConnListener(OpenRTM_aist.ConnectorListener):
  def __init__(self, name):
    self._name = name

  def __del__(self):
    print "dtor of ", self._name

  def __call__(self, info):
    print "------------------------------"
    print "Listener:       ", self._name
    print "Profile::name:  ", info.name
    print "Profile::id:    ", info.id
    print "------------------------------"



class ConsoleOut(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return

  def onInitialize(self):
    self._data = RTC.TimedLong(RTC.Time(0,0),0)
    self._inport = OpenRTM_aist.InPort("in", self._data)
    # Set InPort buffer
    self.addInPort("in", self._inport)

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE,
                                          DataListener("ON_BUFFER_WRITE"))


    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_FULL, 
                                          DataListener("ON_BUFFER_FULL"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE_TIMEOUT, 
                                          DataListener("ON_BUFFER_WRITE_TIMEOUT"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_OVERWRITE, 
                                          DataListener("ON_BUFFER_OVERWRITE"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_READ, 
                                          DataListener("ON_BUFFER_READ"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_SEND, 
                                          DataListener("ON_SEND"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVED,
                                          DataListener("ON_RECEIVED"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_FULL, 
                                          DataListener("ON_RECEIVER_FULL"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_TIMEOUT, 
                                          DataListener("ON_RECEIVER_TIMEOUT"))

    self._inport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_ERROR,
                                          DataListener("ON_RECEIVER_ERROR"))

    self._inport.addConnectorListener(OpenRTM_aist.ConnectorListenerType.ON_CONNECT,
                                      ConnListener("ON_CONNECT"))
    self._inport.addConnectorListener(OpenRTM_aist.ConnectorListenerType.ON_DISCONNECT,
                                      ConnListener("ON_DISCONNECT"))

    return RTC.RTC_OK

  def onExecute(self, ec_id):

    if self._inport.isNew():
      data = self._inport.read()
      print "Received: ", data
      print "Received: ", data.data
      print "TimeStamp: ", data.tm.sec, "[s] ", data.tm.nsec, "[ns]"

    return RTC.RTC_OK


def ConsoleOutInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=consoleout_spec)
  manager.registerFactory(profile,
                          ConsoleOut,
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  ConsoleOutInit(manager)

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
