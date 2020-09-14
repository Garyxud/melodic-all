#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys

import OpenRTM
import RTC
import OpenRTM_aist

consolein_spec = ["implementation_id", "ConsoleIn",
                  "type_name",         "ConsoleIn",
                  "description",       "Console input component",
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


class ConsoleIn(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_INITIALIZE,
                                       self.preOnInitializeFunc)
    self.addExecutionContextActionListener(OpenRTM_aist.ExecutionContextActionListenerType.EC_ATTACHED,
                                           self.onAttachExecutionContextFunc)
    self.addExecutionContextActionListener(OpenRTM_aist.ExecutionContextActionListenerType.EC_DETACHED,
                                           self.onDetachExecutionContextFunc)

    return

  def preOnInitializeFunc(self, ec_id):
    print "preOnInitializeFunc"
    return

  def preOnFinalizeFunc(self, ec_id):
    print "preOnFinalizeFunc"
    return

  def preOnStartupFunc(self, ec_id):
    print "preOnStartupFunc"
    return

  def preOnShutdownFunc(self, ec_id):
    print "preOnShutdownFunc"
    return

  def preOnActivatedFunc(self, ec_id):
    print "preOnActivatedFunc"
    return

  def preOnDeactivatedFunc(self, ec_id):
    print "preOnDeactivatedFunc"
    return

  def preOnAbortingFunc(self, ec_id):
    print "preOnAbortingFunc"
    return

  def preOnErrorFunc(self, ec_id):
    print "preOnErrorFunc"
    return

  def preOnResetFunc(self, ec_id):
    print "preOnResetFunc"
    return

  def preOnExecuteFunc(self, ec_id):
    print "preOnExecuteFunc"
    return

  def preOnStateUpdateFunc(self, ec_id):
    print "preOnStateUpdateFunc"
    return
    
  def preOnRateChangedFunc(self, ec_id):
    print "preOnRateChangedFunc"
    return
    
  def postOnInitializeFunc(self, ec_id, ret):
    print "postOnInitializeFunc, ret: ", ret
    return
    
  def postOnFinalizeFunc(self, ec_id, ret):
    print "postOnFinalizeFunc, ret: ", ret
    return
    
  def postOnStartupFunc(self, ec_id, ret):
    print "postOnStartupFunc, ret: ", ret
    return
    
  def postOnShutdownFunc(self, ec_id, ret):
    print "postOnShutdownFunc, ret: ", ret
    return
    
  def postOnActivatedFunc(self, ec_id, ret):
    print "postOnActivatedFunc, ret: ", ret
    return
    
  def postOnDeactivatedFunc(self, ec_id, ret):
    print "postOnDeactivatedFunc, ret: ", ret
    return
    
  def postOnAbortingFunc(self, ec_id, ret):
    print "postOnAbortingFunc, ret: ", ret
    return
    
  def postOnErrorFunc(self, ec_id, ret):
    print "postOnErrorFunc, ret: ", ret
    return
    
  def postOnResetFunc(self, ec_id, ret):
    print "postOnResetFunc, ret: ", ret
    return
    
  def postOnExecuteFunc(self, ec_id, ret):
    print "postOnExecuteFunc, ret: ", ret
    return
    
  def postOnStateUpdateFunc(self, ec_id, ret):
    print "postOnStateUpdateFunc, ret: ", ret
    return
    
  def postOnRateChangedFunc(self, ec_id, ret):
    print "postOnRateChangedFunc, ret: ", ret
    return
    
  def onAddPortFunc(self, pprof):
    print "onAddPortFunc"
    return
    
  def onRemovePortFunc(self, pprof):
    print "onRemovePortFunc"
    return
    
  def onAttachExecutionContextFunc(self, ec_id):
    print "onAttachExecutionContextFunc"
    return
    
  def onDetachExecutionContextFunc(self, ec_id):
    print "onDetachExecutionContextFunc"
    return


  def onNotifyConnectFunc(self, pname, prof):
    print "onNotifyConnectFunc pname: ",pname
    return

  def onNotifyDisconnectFunc(self, pname, prof):
    print "onNotifyDisconnectFunc pname: ",pname
    return

  def onUnsubscribeInterfacesFunc(self, pname, profile):
    print "onUnsubscribeInterfacesFunc pname: ", pname
    return

  def onPublishInterfacesFunc(self, portname, profile, ret):
    print "onPublishInterfacesFunc pname: ", portname, " ret: ", ret
    return

  def onConnectNextportFunc(self, portname, profile, ret):
    print "onConnectNextportFunc pname: ", portname, " ret: ", ret
    return

  def onSubscribeInterfacesFunc(self, portname, profile, ret):
    print "onSubscribeInterfacesFunc pname: ", portname, " ret: ", ret
    return

  def onConnectedFunc(self, portname, profile, ret):
    print "onConnectedFunc pname: ", portname, " ret: ", ret
    return

  def onDisconnectNextportFunc(self, portname, profile, ret):
    print "onDisconnectNextportFunc pname: ", portname, " ret: ", ret
    return

  def onDisconnectedFunc(self, portname, profile, ret):
    print "onDisconnectedFunc pname: ", portname, " ret: ", ret
    return


  def onInitialize(self):
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_FINALIZE,
                                       self.preOnFinalizeFunc)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_STARTUP,
                                       self.preOnStartupFunc)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_SHUTDOWN,
                                       self.preOnShutdownFunc)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_ACTIVATED,
                                       self.preOnActivatedFunc)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_DEACTIVATED,
                                       self.preOnDeactivatedFunc)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_ABORTING,
                                       self.preOnAbortingFunc)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_ERROR,
                                       self.preOnErrorFunc)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_RESET,
                                       self.preOnResetFunc)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_EXECUTE,
                                       self.preOnExecuteFunc)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_STATE_UPDATE,
                                       self.preOnStateUpdateFunc)
    self.addPreComponentActionListener(OpenRTM_aist.PreComponentActionListenerType.PRE_ON_RATE_CHANGED,
                                       self.preOnRateChangedFunc)

    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_INITIALIZE,
                                        self.postOnInitializeFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_FINALIZE,
                                        self.postOnFinalizeFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_STARTUP,
                                        self.postOnStartupFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_SHUTDOWN,
                                        self.postOnShutdownFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_ACTIVATED,
                                        self.postOnActivatedFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_DEACTIVATED,
                                        self.postOnDeactivatedFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_ABORTING,
                                        self.postOnAbortingFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_ERROR,
                                        self.postOnErrorFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_RESET,
                                        self.postOnResetFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_EXECUTE,
                                        self.postOnExecuteFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_STATE_UPDATE,
                                        self.postOnStateUpdateFunc)
    self.addPostComponentActionListener(OpenRTM_aist.PostComponentActionListenerType.POST_ON_RATE_CHANGED,
                                        self.postOnRateChangedFunc)

    self.addPortActionListener(OpenRTM_aist.PortActionListenerType.ADD_PORT,
                               self.onAddPortFunc)
    self.addPortActionListener(OpenRTM_aist.PortActionListenerType.REMOVE_PORT,
                               self.onRemovePortFunc)


    self.addPortConnectListener(OpenRTM_aist.PortConnectListenerType.ON_NOTIFY_CONNECT,
                                self.onNotifyConnectFunc)
    self.addPortConnectListener(OpenRTM_aist.PortConnectListenerType.ON_NOTIFY_DISCONNECT,
                                self.onNotifyDisconnectFunc)
    self.addPortConnectListener(OpenRTM_aist.PortConnectListenerType.ON_UNSUBSCRIBE_INTERFACES,
                                self.onUnsubscribeInterfacesFunc)

    self.addPortConnectRetListener(OpenRTM_aist.PortConnectRetListenerType.ON_PUBLISH_INTERFACES,
                                   self.onPublishInterfacesFunc)
    self.addPortConnectRetListener(OpenRTM_aist.PortConnectRetListenerType.ON_CONNECT_NEXTPORT,
                                   self.onConnectNextportFunc)
    self.addPortConnectRetListener(OpenRTM_aist.PortConnectRetListenerType.ON_SUBSCRIBE_INTERFACES,
                                   self.onSubscribeInterfacesFunc)
    self.addPortConnectRetListener(OpenRTM_aist.PortConnectRetListenerType.ON_CONNECTED,
                                   self.onConnectedFunc)
    self.addPortConnectRetListener(OpenRTM_aist.PortConnectRetListenerType.ON_DISCONNECT_NEXT,
                                   self.onDisconnectNextportFunc)
    self.addPortConnectRetListener(OpenRTM_aist.PortConnectRetListenerType.ON_DISCONNECTED,
                                   self.onDisconnectedFunc)


    self._data = RTC.TimedLong(RTC.Time(0,0),0)
    self._outport = OpenRTM_aist.OutPort("out", self._data)
    # Set OutPort buffer
    self.addOutPort("out", self._outport)
    self._outport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE,
                                           DataListener("ON_BUFFER_WRITE"))
    self._outport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_FULL, 
                                           DataListener("ON_BUFFER_FULL"))
    self._outport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE_TIMEOUT, 
                                           DataListener("ON_BUFFER_WRITE_TIMEOUT"))
    self._outport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_OVERWRITE, 
                                           DataListener("ON_BUFFER_OVERWRITE"))
    self._outport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_READ, 
                                           DataListener("ON_BUFFER_READ"))
    self._outport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_SEND, 
                                           DataListener("ON_SEND"))
    self._outport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVED,
                                           DataListener("ON_RECEIVED"))
    self._outport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_FULL, 
                                           DataListener("ON_RECEIVER_FULL"))
    self._outport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_TIMEOUT, 
                                           DataListener("ON_RECEIVER_TIMEOUT"))
    self._outport.addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_ERROR,
                                           DataListener("ON_RECEIVER_ERROR"))

    self._outport.addConnectorListener(OpenRTM_aist.ConnectorListenerType.ON_CONNECT,
                                       ConnListener("ON_CONNECT"))
    self._outport.addConnectorListener(OpenRTM_aist.ConnectorListenerType.ON_DISCONNECT,
                                       ConnListener("ON_DISCONNECT"))

    return RTC.RTC_OK

        
  def onExecute(self, ec_id):
    print "Please input number: ",
    self._data.data = long(sys.stdin.readline())
    if self._data.data == 9:
      return RTC.RTC_ERROR
    OpenRTM_aist.setTimestamp(self._data)
    print "Sending to subscriber: ", self._data.data
    self._outport.write()
    return RTC.RTC_OK


def ConsoleInInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=consolein_spec)
  manager.registerFactory(profile,
                          ConsoleIn,
                          OpenRTM_aist.Delete)


def MyModuleInit(manager):
  ConsoleInInit(manager)

  # Create a component
  comp = manager.createComponent("ConsoleIn")

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
