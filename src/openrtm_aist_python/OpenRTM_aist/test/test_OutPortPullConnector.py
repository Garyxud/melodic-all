#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_OutPortPullConnector.py
#  \brief test for OutPortPullConnector class
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
from OutPortPullConnector import *

import RTC, RTC__POA

import OpenRTM_aist

class MyBuffer:
  def __init__(self):
    self._data = None
    return

  def write(self, data):
    self._data = data
    return
    
  def read(self):
    return self._data

  def init(self,info):
    pass

class OutPortProviderMock:
  _buffer = None
  _prop = None

  def init(self, prop):
    self._prop = prop

  def setBuffer(self, buff):
    self._buffer = buff

  def setListener(self, info, listener):
    pass

  def setConnector(self, con):
    pass


class TestOutPortPullConnector(unittest.TestCase):
  def setUp(self):
    self._buffer = MyBuffer()
    self._profile = OpenRTM_aist.ConnectorInfo("test",
                                               "id",
                                               ["in","out"],
                                               OpenRTM_aist.Properties())

    self._oc = OutPortPullConnector(self._profile,OutPortProviderMock(),OpenRTM_aist.ConnectorListeners(), self._buffer)
    return

  def test_write(self):
    self._oc.write(123)
    # self.assertEqual(self._buffer.read(), 123)
    return


  def test_disconnect(self):
    self.assertEqual(self._oc.disconnect(), OpenRTM_aist.DataPortStatus.PORT_OK)
    return


  def test_getBuffer(self):
    self.assertEqual(self._oc.getBuffer(),self._buffer)
    return


############### test #################
if __name__ == '__main__':
  unittest.main()
