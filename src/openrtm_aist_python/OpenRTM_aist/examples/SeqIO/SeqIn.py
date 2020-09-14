#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

import sys
import time

import RTC
import OpenRTM_aist

seqin_spec = ["implementation_id", "SeqIn",
              "type_name",         "SequenceInComponent",
              "description",       "Sequence InPort component",
              "version",           "1.0",
              "vendor",            "Shinji Kurihara",
              "category",          "example",
              "activity_type",     "DataFlowComponent",
              "max_instance",      "10",
              "language",          "Python",
              "lang_type",         "script",
              ""]


class SeqIn(OpenRTM_aist.DataFlowComponentBase):
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

    self._octetIn     = OpenRTM_aist.InPort("Octet", self._octet)
    self._shortIn     = OpenRTM_aist.InPort("Short", self._short)
    self._longIn      = OpenRTM_aist.InPort("Long", self._long)
    self._floatIn     = OpenRTM_aist.InPort("Float", self._float)
    self._doubleIn    = OpenRTM_aist.InPort("Double", self._double)
    self._octetSeqIn  = OpenRTM_aist.InPort("OctetSeq", self._octetSeq)
    self._shortSeqIn  = OpenRTM_aist.InPort("ShortSeq", self._shortSeq)
    self._longSeqIn   = OpenRTM_aist.InPort("LongSeq", self._longSeq)
    self._floatSeqIn  = OpenRTM_aist.InPort("FloatSeq", self._floatSeq)
    self._doubleSeqIn = OpenRTM_aist.InPort("DoubleSeq", self._doubleSeq)


    # Set InPort buffer
    self.addInPort("Octet", self._octetIn)
    self.addInPort("Short", self._shortIn)
    self.addInPort("Long", self._longIn)
    self.addInPort("Float", self._floatIn)
    self.addInPort("Double", self._doubleIn)

    self.addInPort("OctetSeq", self._octetSeqIn)
    self.addInPort("ShortSeq", self._shortSeqIn)
    self.addInPort("LongSeq", self._longSeqIn)
    self.addInPort("FloatSeq", self._floatSeqIn)
    self.addInPort("DoubleSeq", self._doubleSeqIn)
    return RTC.RTC_OK


  def onExecute(self, ec_id):
    octet_  = self._octetIn.read()
    short_  = self._shortIn.read()
    long_   = self._longIn.read()
    float_  = self._floatIn.read()
    double_ = self._doubleIn.read()

    octetSeq_  = self._octetSeqIn.read()
    shortSeq_  = self._shortSeqIn.read()
    longSeq_   = self._longSeqIn.read()
    floatSeq_  = self._floatSeqIn.read()
    doubleSeq_ = self._doubleSeqIn.read()

    octetSize_  = len(octetSeq_.data)
    shortSize_  = len(shortSeq_.data)
    longSize_   = len(longSeq_.data)
    floatSize_  = len(floatSeq_.data)
    doubleSize_ = len(doubleSeq_.data)

    octetSeqDisp_ = []
    for i in range(octetSize_):
      octetSeqDisp_.append(ord(octetSeq_.data[i]))

    maxsize = max(octetSize_, shortSize_, longSize_, floatSize_, doubleSize_)
    octetSeqDisp_   = octetSeqDisp_   + ['-'] * (maxsize - octetSize_)
    shortSeq_.data  = shortSeq_.data  + ['-'] * (maxsize - shortSize_)
    longSeq_.data   = longSeq_.data   + ['-'] * (maxsize - longSize_)
    floatSeq_.data  = floatSeq_.data  + ['-'] * (maxsize - floatSize_)
    doubleSeq_.data = doubleSeq_.data + ['-'] * (maxsize - doubleSize_)

    if 0x20 <= octet_.data < 0x7e :
      octetDisp_ = chr(octet_.data)
    else:
      octetDisp_ = " "
    print '%3.2s %10.8s %10.8s %10.8s %10.8s %10.8s' \
        % (' ', 'octet', 'short', 'long', 'float', 'double')
    print '%3.2s %7s[%s] %10.8s %10.8s %10.8s %10.8s' \
        % (' ', octet_.data, octetDisp_, short_.data, long_.data, float_.data, double_.data)
    print "-----------------------------------------------------------"
    print "                 Sequence Data                     "
    print "-----------------------------------------------------------"
    for i in range(maxsize):
      if 0x20 <= octetSeqDisp_[i] < 0x7e :
        octetDisp_ = chr(octetSeqDisp_[i])
      else:
        octetDisp_ = " "
      print '%3.2s %7s[%s] %10.8s %10.8s %10.8s %10.8s' \
          % (i, octetSeqDisp_[i], octetDisp_, shortSeq_.data[i], longSeq_.data[i], floatSeq_.data[i], doubleSeq_.data[i])

    # Moving cursor (^[[nA : n lines upward)
    print "\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r"

    time.sleep(0.5)
    return RTC.RTC_OK


def SeqInInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=seqin_spec)
  manager.registerFactory(profile,
                          SeqIn,
                          OpenRTM_aist.Delete)


def MyModuleInit(manager):
  SeqInInit(manager)

  # Create a component
  comp = manager.createComponent("SeqIn")


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

