#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys

import RTC
import OpenRTM_aist

import random
import time

seqout_spec = ["implementation_id", "SeqOut",
               "type_name",         "SequenceOutComponent",
               "description",       "Sequence OutPort component",
               "version",           "1.0",
               "vendor",            "Shinji Kurihara",
               "category",          "example",
               "activity_type",     "DataFlowComponent",
               "max_instance",      "10",
               "language",          "Python",
               "lang_type",         "script",
               ""]



class SeqOut(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return
    
  def onInitialize(self):
    self._octet     = RTC.TimedOctet(RTC.Time(0,0),0)
    self._short     = RTC.TimedShort(RTC.Time(0,0),0)
    self._long      = RTC.TimedLong(RTC.Time(0,0),0)
    self._float     = RTC.TimedFloat(RTC.Time(0,0),0)
    self._double    = RTC.TimedDouble(RTC.Time(0,0),0)
    self._octetSeq  = RTC.TimedOctetSeq(RTC.Time(0,0),[])
    self._shortSeq  = RTC.TimedShortSeq(RTC.Time(0,0),[])
    self._longSeq   = RTC.TimedLongSeq(RTC.Time(0,0),[])
    self._floatSeq  = RTC.TimedFloatSeq(RTC.Time(0,0),[])
    self._doubleSeq = RTC.TimedDoubleSeq(RTC.Time(0,0),[])

    self._octetOut     = OpenRTM_aist.OutPort("Octet", self._octet)
    self._shortOut     = OpenRTM_aist.OutPort("Short", self._short)
    self._longOut      = OpenRTM_aist.OutPort("Long", self._long)
    self._floatOut     = OpenRTM_aist.OutPort("Float", self._float)
    self._doubleOut    = OpenRTM_aist.OutPort("Double", self._double)
    self._octetSeqOut  = OpenRTM_aist.OutPort("OctetSeq", self._octetSeq)
    self._shortSeqOut  = OpenRTM_aist.OutPort("ShortSeq", self._shortSeq)
    self._longSeqOut   = OpenRTM_aist.OutPort("LongSeq", self._longSeq)
    self._floatSeqOut  = OpenRTM_aist.OutPort("FloatSeq", self._floatSeq)
    self._doubleSeqOut = OpenRTM_aist.OutPort("DoubleSeq", self._doubleSeq)


    # Set OutPort buffer
    self.addOutPort("Octet", self._octetOut)
    self.addOutPort("Short", self._shortOut)
    self.addOutPort("Long", self._longOut)
    self.addOutPort("Float", self._floatOut)
    self.addOutPort("Double", self._doubleOut)

    self.addOutPort("OctetSeq", self._octetSeqOut)
    self.addOutPort("ShortSeq", self._shortSeqOut)
    self.addOutPort("LongSeq", self._longSeqOut)
    self.addOutPort("FloatSeq", self._floatSeqOut)
    self.addOutPort("DoubleSeq", self._doubleSeqOut)
    return RTC.RTC_OK


  def onExecute(self, ec_id):
    octetSeq  = ""
    shortSeq  = []
    longSeq   = []
    floatSeq  = []
    doubleSeq = []

    self._octet.data = int(random.uniform(0x41, 0x4a))
    self._short.data = int(random.uniform(0, 10))
    self._long.data = long(random.uniform(0, 10))
    self._float.data = float(random.uniform(0.0, 10.0))
    self._double.data = float(random.uniform(0.0, 10.0))

    print '%3.2s   %10.8s %10.8s %10.8s %10.8s %10.8s' \
        % (' ', 'octet', 'short', 'long', 'float', 'double')
    print '%3.2s   %7s[%s] %10.8s %10.8s %10.8s %10.8s' \
        % (' ', self._octet.data, chr(self._octet.data), self._short.data, self._long.data, self._float.data, self._double.data)
    print "-------------------------------------------------------------"
    print "                 Sequence Data                     "
    print "-------------------------------------------------------------"
    for i in range(10):
      octetSeq = octetSeq + chr(int(random.uniform(0x41, 0x4a)))
      shortSeq.append(int(random.uniform(0, 10)))
      longSeq.append(long(random.uniform(0, 10)))
      floatSeq.append(float(random.uniform(0.0, 10.0)))
      doubleSeq.append(float(random.uniform(0.0, 10.0)))
      print '%3.2s : %7s[%s] %10.8s %10.8s %10.8s %10.8s' \
          % (str(i), ord(octetSeq[i]), octetSeq[i], shortSeq[i], longSeq[i], floatSeq[i], doubleSeq[i])

    # Moving cursor (^[[nA : n lines upward)
    print "[A\r[A\r[A\r[A\r[A\r[A\r[A\r[A\r[A\r[A\r[A\r[A\r[A\r[A\r[A\r[A\r"
        
    self._octetSeq.data = octetSeq
    self._shortSeq.data = shortSeq
    self._longSeq.data = longSeq
    self._floatSeq.data = floatSeq
    self._doubleSeq.data = doubleSeq

    self._octetOut.write()
    self._shortOut.write()
    self._longOut.write()
    self._floatOut.write()
    self._doubleOut.write()
    self._octetSeqOut.write()
    self._shortSeqOut.write()
    self._longSeqOut.write()
    self._floatSeqOut.write()
    self._doubleSeqOut.write()

    time.sleep(1)

    return RTC.RTC_OK


def SeqOutInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=seqout_spec)
  manager.registerFactory(profile,
                          SeqOut,
                          OpenRTM_aist.Delete)
  return


def MyModuleInit(manager):
  SeqOutInit(manager)

  # Create a component
  comp = manager.createComponent("SeqOut")

  print "Component created"


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
