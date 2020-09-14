#!/usr/bin/env python
# -*- Python -*-


#  \file test_InPortPushConnector.py
#  \brief test for InPortPushConnector class
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

from InPortPushConnector import *

import RTC, RTC__POA
import OpenRTM_aist
from OpenRTM_aist import *


class InPortProviderMock:
  _buffer = None
  _prop = None

  def init(self, prop):
    self._prop = prop

  def setBuffer(self, buff):
    self._buffer = buff

  def setListener(self, info, listener):
    pass


class TestInPortPushConnector(unittest.TestCase):
  def setUp(self):
    OpenRTM_aist.Manager.instance()
    self._con = InPortPushConnector(OpenRTM_aist.ConnectorInfo("name","id",[],OpenRTM_aist.Properties()),InPortProviderMock(),OpenRTM_aist.ConnectorListeners())
  
  def test_read(self):
    data = []
    self.assertEqual(self._con.read(data),OpenRTM_aist.DataPortStatus.BUFFER_TIMEOUT)
    data = 123
    self.assertEqual(self._con.read(data),OpenRTM_aist.DataPortStatus.BUFFER_TIMEOUT)
    return

  def test_disconnect(self):
    self.assertEqual(self._con.disconnect(),OpenRTM_aist.DataPortStatus.PORT_OK)
    return

  def test_activate(self):
    self._con.activate()
    return

  def test_deactivate(self):
    self._con.deactivate()
    return

  def test_createBuffer(self):
    self._con.createBuffer(OpenRTM_aist.ConnectorInfo("name","id",[],OpenRTM_aist.Properties()))
    return




############### test #################
if __name__ == '__main__':
  unittest.main()

