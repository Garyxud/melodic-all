#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_OutPort.py
#  \brief test for OutPort class
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
from OutPort import *

import RTC, RTC__POA

import OpenRTM_aist

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

class ConnectorMock:
  def write(self, data):
    self._data = data
    return OpenRTM_aist.DataPortStatus.PORT_OK

  def read(self, data):
    data[0] = self._data
    return True


class TestOutPort(unittest.TestCase):
  def setUp(self):
    OpenRTM_aist.Manager.init(sys.argv)
    self._op = OutPort("out", RTC.TimedLong(RTC.Time(0,0), 0))
    return

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_write(self):
    self.assertEqual(self._op.write(RTC.TimedLong(RTC.Time(0,0), 123)), True)
    self._connector = ConnectorMock()
    self._op._connectors = [self._connector]
    self.assertEqual(self._op.write(RTC.TimedLong(RTC.Time(0,0), 123)), True)
    read_data = [RTC.TimedLong(RTC.Time(0,0), 0)]
    self._connector.read(read_data)
    self.assertEqual(read_data[0].data,123)
    return

  def test_OnWrite(self):
    self._connector = ConnectorMock()
    self._op._connectors = [self._connector]
    self._op.setOnWrite(OnRWTest().echo)
    self._op.setOnWriteConvert(OnRWConvertTest().echo)
    self.assertEqual(self._op.write(RTC.TimedLong(RTC.Time(0,0), 123)), True)
    return


  def test_getPortDataType(self):
    self.assertEqual(self._op.getPortDataType(),any.to_any(RTC.TimedLong(RTC.Time(0,0),0)).typecode().name())
    return


############### test #################
if __name__ == '__main__':
  unittest.main()
