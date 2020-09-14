#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_OutPortPushConnector.py
#  \brief test for OutPortPushConnector class
#  \date $Date: 2007/09/19$
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Noriaki Ando
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 

import sys
sys.path.insert(1,"../")

from omniORB import *
from omniORB import any

import unittest
from OutPortPushConnector import *

import OpenRTM
import RTC
import OpenRTM_aist

class MyBuffer:
  def __init__(self):
    self._data = None
    return

  def write(self, data, sec=0, usec=0):
    self._data = data
    return
    
  def read(self):
    return self._data

  def init(self,info):
    return

class ConsumerMock:
  def __init__(self,buff):
    self._data = None
    self._buff = buff
    return

  def init(self,prop):
    return

  def write(self, data):
    self._data = data
    return OpenRTM.PORT_OK

  def put(self, data):
    self._buff.write(data)
    return OpenRTM.PORT_OK

class TestOutPortPushConnector(unittest.TestCase):
  def setUp(self):
    self._buffer = MyBuffer()
    self._consumer = ConsumerMock(self._buffer)
    self._profile = OpenRTM_aist.ConnectorInfo("test",
                                               "id",
                                               ["in","out"],
                                               OpenRTM_aist.Properties())

    self._oc = OutPortPushConnector(self._profile,self._consumer,OpenRTM_aist.ConnectorListeners(),self._buffer)
    return

  def test_write(self):
    wdata = RTC.TimedLong(RTC.Time(0,0), 123)
    self._oc.write(wdata)
    val = self._buffer.read()
    rdata = RTC.TimedLong(RTC.Time(0,0), 0)
    get_data = cdrUnmarshal(any.to_any(rdata).typecode(),val,1)
    self.assertEqual(get_data.data, 123)
    return


  def test_disconnect(self):
    self.assertEqual(self._oc.disconnect(), OpenRTM_aist.DataPortStatus.PORT_OK)
    return


  def test_activate(self):
    self._oc.activate()
    return

  def test_deactivate(self):
    self._oc.deactivate()
    return

  def test_getBuffer(self):
    self.assertEqual(self._oc.getBuffer(),self._buffer)
    return

  def test_createPublisher(self):
    self._oc.createPublisher(self._profile)
    return

  def test_createBuffer(self):
    self._oc.createBuffer(self._profile)
    return

############### test #################
if __name__ == '__main__':
        unittest.main()
