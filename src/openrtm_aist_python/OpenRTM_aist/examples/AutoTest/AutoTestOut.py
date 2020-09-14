#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 \file AutoTestOut.py
 \brief ModuleDescription
 \date $Date$


"""
import sys
import string
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
import AutoTest, AutoTest__POA
from omniORB import CORBA

#import csv

# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
AutoTestOut_spec = ["implementation_id", "AutoTestOut", 
                    "type_name",         "AutoTestOut", 
                    "description",       "ModuleDescription", 
                    "version",           "1.0.0", 
                    "vendor",            "HarumiMiyamoto", 
                    "category",          "example", 
                    "activity_type",     "STATIC", 
                    "max_instance",      "1", 
                    "language",          "Python", 
                    "lang_type",         "SCRIPT",
                    "exec_cxt.periodic.rate", "1.0",
                    ""]


class AutoTestOut(OpenRTM_aist.DataFlowComponentBase):
  
  """
  \class AutoTestOut
  \brief ModuleDescription
  """
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return
        
  def onInitialize(self):
    self._d_dp_vOut = RTC.TimedFloat(RTC.Time(0,0),0)
    self._d_dp_vSeqOut = RTC.TimedFloatSeq(RTC.Time(0,0),[])
    
    self._OutOut = OpenRTM_aist.OutPort("out", self._d_dp_vOut, OpenRTM_aist.RingBuffer(8))
    self._SeqOutOut = OpenRTM_aist.OutPort("seqout", self._d_dp_vSeqOut, OpenRTM_aist.RingBuffer(8))
  
    # Set OutPort buffers
    self.addOutPort("out", self._OutOut)
    self.addOutPort("seqout", self._SeqOutOut)
        
    self._MyServicePort = OpenRTM_aist.CorbaPort("MyService")
    self._myservice0_var = OpenRTM_aist.CorbaConsumer(interfaceType=AutoTest.MyService)
    
    # Set service consumers to Ports
    self._MyServicePort.registerConsumer("myservice0", "MyService", self._myservice0_var)
    
    # Set CORBA Service Ports
    self.addPort(self._MyServicePort)
  
    return RTC.RTC_OK
  
  def onActivated(self, ec_id):
    try:
      self._file=open('original-data')
    except:
      print "Can not open 'original-data' file."
      return RTC.RTC_ERROR

    return RTC.RTC_OK
  
  def onDeactivated(self, ec_id): 
    self._file.close()
    return RTC.RTC_OK
  

  def onExecute(self, ec_id):
    floatSeq=[]
    str1 = self._file.readline()
    str2 = self._file.readline()
    str3 = self._file.readline()
    
    if str1:  
      self._d_dp_vOut.data = float(str1)

      if str2:
        str2 = str2.split(' ')
        floatSeq.append(float(str2[0]))
        floatSeq.append(float(str2[1]))
        floatSeq.append(float(str2[2]))
        floatSeq.append(float(str2[3]))
        floatSeq.append(float(str2[4]))
        self._d_dp_vSeqOut.data = floatSeq

        if str3:
          if self._myservice0_var._ptr():
            # Write out data
            self._OutOut.write()
            self._SeqOutOut.write()

            # invoking echo operation
            retmsg = self._myservice0_var._ptr().echo(str3.rstrip('\r\n'))
          
    return RTC.RTC_OK
        

def AutoTestOutInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=AutoTestOut_spec)
  manager.registerFactory(profile,
                          AutoTestOut,
                          OpenRTM_aist.Delete)

def MyModuleInit(manager):
  AutoTestOutInit(manager)

  # Create a component
  comp = manager.createComponent("AutoTestOut")



def main():
  mgr = OpenRTM_aist.Manager.init(len(sys.argv), sys.argv)
  mgr.setModuleInitProc(MyModuleInit)
  mgr.activateManager()
  mgr.runManager()

if __name__ == "__main__":
  
  main()

