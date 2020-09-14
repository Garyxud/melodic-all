#!/usr/bin/env python
# -*- Python -*-

##
# @file test_PortConnectListener.py
# @brief test for port connector listener class
# @date $Date: 2011/03/18 $
# @author Shinji Kurihara
#
# Copyright (C) 2011
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.


import sys
sys.path.insert(1,"../")

import unittest
import OpenRTM_aist
from PortConnectListener import *

import RTC, RTC__POA
import OpenRTM

from omniORB import *
from omniORB import any

class MockPortConnectListener(PortConnectListener):
  def __init__(self):
    PortConnectListener.__init__(self)
    return

  def __call__(self,portname,profile):
    return


class MockPortConnectRetListener(PortConnectRetListener):
  def __init__(self):
    PortConnectRetListener.__init__(self)
    return

  def __call__(self,portname,profile,ret):
    return



class TestListener(unittest.TestCase):
  def setUp(self):
    return

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_PortConnectListener_toString(self):
    self.assertEqual("ON_NOTIFY_CONNECT",
                     PortConnectListener.toString(
        PortConnectListenerType.ON_NOTIFY_CONNECT))
    
    self.assertEqual("ON_NOTIFY_DISCONNECT",
                     PortConnectListener.toString(
        PortConnectListenerType.ON_NOTIFY_DISCONNECT))

    self.assertEqual("ON_UNSUBSCRIBE_INTERFACES",
                     PortConnectListener.toString(
        PortConnectListenerType.ON_UNSUBSCRIBE_INTERFACES))
    return


  def test_PortConnectRetListener_toString(self):
    self.assertEqual("ON_PUBLISH_INTERFACES",
                     PortConnectRetListener.toString(
        PortConnectRetListenerType.ON_PUBLISH_INTERFACES))
    
    self.assertEqual("ON_CONNECT_NEXTPORT",
                     PortConnectRetListener.toString(
        PortConnectRetListenerType.ON_CONNECT_NEXTPORT))

    self.assertEqual("ON_SUBSCRIBE_INTERFACES",
                     PortConnectRetListener.toString(
        PortConnectRetListenerType.ON_SUBSCRIBE_INTERFACES))

    self.assertEqual("ON_CONNECTED",
                     PortConnectRetListener.toString(
        PortConnectRetListenerType.ON_CONNECTED))

    self.assertEqual("ON_DISCONNECT_NEXT",
                     PortConnectRetListener.toString(
        PortConnectRetListenerType.ON_DISCONNECT_NEXT))

    self.assertEqual("ON_DISCONNECTED",
                     PortConnectRetListener.toString(
        PortConnectRetListenerType.ON_DISCONNECTED))
    return


  def test_PortConnectListenerHolder(self):
    portconlisteners = PortConnectListeners()
    listener = MockPortConnectListener()
    portconlisteners.portconnect_[0].addListener(listener,True)
    portconlisteners.portconnect_[0].notify("port_name",None)
    portconlisteners.portconnect_[0].removeListener(listener)
    return

  def test_PortConnectRetListenerHolder(self):
    portconretlisteners = PortConnectRetListeners()
    listener = MockPortConnectRetListener()
    portconretlisteners.portconnret_[0].addListener(listener,True)
    portconretlisteners.portconnret_[0].notify("port_name",None)
    portconretlisteners.portconnret_[0].removeListener(listener)
    return


############### test #################
if __name__ == '__main__':
        unittest.main()

