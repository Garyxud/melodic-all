#!/usr/bin/env python
# -*- Python -*-


#  \file test_InPortPullConnector.py
#  \brief test for InPortPullConnector class
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

from InPortPullConnector import *

import RTC, RTC__POA
import OpenRTM_aist

class InPortConsumerMock:
  _buffer = None

  def setBuffer(self, buff):
    self._buffer = buff

  def setListener(self, info, listener):
    pass

  def get(self, data):
    return OpenRTM_aist.DataPortStatus.PORT_OK

class TestInPortPullConnector(unittest.TestCase):
  def setUp(self):
    OpenRTM_aist.Manager.instance()
    self._con = InPortPullConnector(OpenRTM_aist.ConnectorInfo("name","id",[],OpenRTM_aist.Properties()),InPortConsumerMock(),OpenRTM_aist.ConnectorListeners())
  
  def test_read(self):
    data = []
    self.assertEqual(self._con.read(data),OpenRTM_aist.BufferStatus.PRECONDITION_NOT_MET)
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

