#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import string

import RTC
import SimpleService
import OpenRTM_aist
from omniORB import CORBA

myserviceconsumer_spec = ["implementation_id", "MyServiceConsumer",
                          "type_name",         "MyServiceConsumer",
                          "description",       "MyService Consumer Sample component",
                          "version",           "1.0",
                          "vendor",            "Shinji Kurihara",
                          "category",          "example",
                          "activity_type",     "DataFlowComponent",
                          "max_instance",      "10",
                          "language",          "Python",
                          "lang_type",         "script",
                          ""]

class echo_functor:
  def __init__(self, msg, result):
    self._msg = msg
    self._result = result
    return

  def __call__(self, obj):
    try:
      if CORBA.is_nil(obj):
        print "No service connected."
      else:
        self._result[0] = obj.echo(self._msg)
    except:
      pass


class MyServiceConsumer(OpenRTM_aist.DataFlowComponentBase):
  # constructor
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

    self._async_echo = None
    self._result = [None]
    return

  def onInitialize(self):
    # initialization of CORBA Port
    self._myServicePort = OpenRTM_aist.CorbaPort("MyService")

    # initialization of Consumer
    self._myservice0 = OpenRTM_aist.CorbaConsumer(interfaceType=SimpleService.MyService)
        
    # Set service consumers to Ports
    self._myServicePort.registerConsumer("myservice0", "MyService", self._myservice0)

    # Set CORBA Service Ports
    self.addPort(self._myServicePort)

    return RTC.RTC_OK

  # The execution action that is invoked periodically
  def onExecute(self, ec_id):
    print "\n"
    print "Command list: "
    print " echo [msg]       : echo message."
    print " set_value [value]: set value."
    print " get_value        : get current value."
    print " get_echo_history : get input messsage history."
    print " get_value_history: get input value history."
    print "> ",

    args = str(sys.stdin.readline())
    argv = string.split(args)
    argv[-1] = argv[-1].rstrip("\n")

    if self._async_echo and self._async_echo.finished():
      print "echo() finished: ", self._result[0]
      self._async_echo = None

    if argv[0] == "echo" and len(argv) > 1:
      if not self._async_echo:
        retmsg = ""
        func = echo_functor(argv[1],self._result)
        self._async_echo = OpenRTM_aist.Async_tInvoker(self._myservice0._ptr(),
                                                               func)
        self._async_echo.invoke()
      else:
        print "echo() still invoking"

      return RTC.RTC_OK

    if argv[0] == "set_value" and len(argv) > 1:
      val = float(argv[1])
      self._myservice0._ptr().set_value(val)
      print "Set remote value: ", val
      return RTC.RTC_OK
      
    if argv[0] == "get_value":
      retval = self._myservice0._ptr().get_value()
      print "Current remote value: ", retval
      return RTC.RTC_OK;
      
    if argv[0] == "get_echo_history":
      OpenRTM_aist.CORBA_SeqUtil.for_each(self._myservice0._ptr().get_echo_history(),
                                          self.seq_print())
      return RTC.RTC_OK
      
    if argv[0] == "get_value_history":
      OpenRTM_aist.CORBA_SeqUtil.for_each(self._myservice0._ptr().get_value_history(),
                                          self.seq_print())
      return RTC.RTC_OK
      
    print "Invalid command or argument(s)."

    return RTC.RTC_OK


  # functor class to print sequence data
  class seq_print:
    def __init__(self):
      self._cnt = 0
      return

    def __call__(self, val):
      print self._cnt, ": ", val
      self._cnt += 1
      return


def MyServiceConsumerInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=myserviceconsumer_spec)
  manager.registerFactory(profile,
                          MyServiceConsumer,
                          OpenRTM_aist.Delete)


def MyModuleInit(manager):
  MyServiceConsumerInit(manager)

  # Create a component
  comp = manager.createComponent("MyServiceConsumer")
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
  return


if __name__ == "__main__":
  main()
