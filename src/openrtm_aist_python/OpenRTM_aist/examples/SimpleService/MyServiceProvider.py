#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import string
import time

import RTC
import SimpleService, SimpleService__POA
import OpenRTM_aist

# Module specification
myserviceprovider_spec = ["implementation_id", "MyServiceProvider",
                          "type_name",         "MyServiceProvider",
                          "description",       "MyService Provider Sample component",
                          "version",           "1.0",
                          "vendor",            "Shinji Kurihara",
                          "category",          "example",
                          "activity_type",     "DataFlowComponent",
                          "max_instance",      "10",
                          "language",          "Python",
                          "lang_type",         "script",
                          ""]

# functor class to print sequence data
class seq_print:
  def __init__(self):
    self._cnt = 0
    return

  def __call__(self, val):
    print self._cnt, ": ", val
    self._cnt += 1
    return


# Class implementing IDL interface MyService(MyService.idl)
class MyServiceSVC_impl(SimpleService__POA.MyService):
  def __init__(self):
    self._echoList = []
    self._valueList = []
    self._value = 0
    return

  def __del__(self):
    pass

  def echo(self, msg):
    OpenRTM_aist.CORBA_SeqUtil.push_back(self._echoList, msg)
    print "MyService::echo() was called."
    for i in range(10):
      print "Message: ", msg
      time.sleep(1)
    print "MyService::echo() was finished."
    return msg

  def get_echo_history(self):
    print "MyService::get_echo_history() was called."
    OpenRTM_aist.CORBA_SeqUtil.for_each(self._echoList, seq_print())
    return self._echoList

  def set_value(self, value):
    OpenRTM_aist.CORBA_SeqUtil.push_back(self._valueList, value)
    self._value = value
    print "MyService::set_value() was called."
    print "Current value: ", self._value
    return

  def get_value(self):
    print "MyService::get_value() was called."
    print "Current value: ", self._value
    return float(self._value)

  def get_value_history(self):
    print "MyService::get_value_history() was called."
    OpenRTM_aist.CORBA_SeqUtil.for_each(self._valueList, seq_print())

    return self._valueList



class MyServiceProvider(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

    return
        

  def onInitialize(self):
    # initialization of CORBA Port
    self._myServicePort = OpenRTM_aist.CorbaPort("MyService")

    # initialization of Provider
    self._myservice0 = MyServiceSVC_impl()

    # Set service providers to Ports
    self._myServicePort.registerProvider("myservice0", "MyService", self._myservice0)

    # Set CORBA Service Ports
    self.addPort(self._myServicePort)

    return RTC.RTC_OK


def MyServiceProviderInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=myserviceprovider_spec)
  manager.registerFactory(profile,
                          MyServiceProvider,
                          OpenRTM_aist.Delete)
  return


def MyModuleInit(manager):
  MyServiceProviderInit(manager)

  # Create a component
  comp = manager.createComponent("MyServiceProvider")

  """
  rtobj = manager.getPOA().servant_to_reference(comp)._narrow(RTC.RTObject)

  ecs = rtobj.get_execution_context_services()
  ecs[0].activate_component(rtobj)
  """
  return


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
