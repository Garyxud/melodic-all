#!/usr/bin/env python
# -*- Python -*-


#  \file test_InPort.py
#  \brief test for InPort template class
#  \date $Date: 2007/09/20 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2003-2005
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
 

import sys
sys.path.insert(1,"../")

import unittest

from InPort import *

import RTC, RTC__POA
import OpenRTM_aist

ShareCount = 0

class OnRWTest:
  def __init__(self):
    pass

  def echo(self, value=None):
    print "OnRW Called"


class OnRWConvertTest:
  def __init__(self):
    pass

  def echo(self, value=None):
    print "OnRWConvert Called"
    return value


class BufferMock:
  def __init__(self):
    self._ShareCount = 0
    self._data = None
    return

  def write(self, data):
    global ShareCount
    ShareCount += 1
    self._ShareCount = ShareCount
    self._data = data
    return OpenRTM_aist.BufferStatus.BUFFER_OK

  def read(self, value):
    global ShareCount
    ShareCount -= 1
    if ShareCount < 0:
      ShareCount = 0
    self._ShareCount = ShareCount
    if len(value) > 0:
      value[0] = self._data
    else:
      value.append(self._data)
    return OpenRTM_aist.BufferStatus.BUFFER_OK

  def readable(self):
    return self._ShareCount


class ConnectorMock:
  def __init__(self, buffer):
    self._buffer = buffer
    return

  def write(self, data):
    self._buffer.write(data)
    return OpenRTM_aist.BufferStatus.BUFFER_OK

  def read(self, data):
    self._buffer.read(data)
    return OpenRTM_aist.DataPortStatus.PORT_OK

  def getBuffer(self):
    ret = self._buffer.readable()
    return self._buffer


class TestInPort(unittest.TestCase):
  def setUp(self):
    OpenRTM_aist.Manager.instance()
    self._buffer = BufferMock()
    self._ipn = InPort("in", RTC.TimedLong(RTC.Time(0,0), 0), self._buffer)
    self._connector = ConnectorMock(self._buffer)
    self._ipn._connectors = [self._connector]

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_name(self):
    self.assertEqual(self._ipn.name(), "in")

  def test_read(self):
    self.assertEqual(self._ipn.isEmpty(), True)
    self.assertEqual(self._ipn.isNew(), False)
    self._connector.write(RTC.TimedLong(RTC.Time(0,0), 123))
    self.assertEqual(self._ipn.isEmpty(), False)
    self.assertEqual(self._ipn.isNew(), True)
    read_data = self._ipn.read()
    self.assertEqual(self._ipn.isEmpty(), True)
    self.assertEqual(self._ipn.isNew(), False)
    self.assertEqual(read_data.data, 123)
    self._ipn.update()
    return

  def test_OnRead(self):
    self._connector.write(RTC.TimedLong(RTC.Time(0,0), 456))
    self._ipn.setOnRead(OnRWTest().echo)
    self._ipn.setOnReadConvert(OnRWConvertTest().echo)
    read_data = self._ipn.read()
    self.assertEqual(read_data.data, 456)
    return


############### test #################
if __name__ == '__main__':
  unittest.main()

