#!/usr/bin/env python
# -*- Python -*-

#
# \file  test_OutPortProvider.py
# \brief test for OutPortProvider class
# \date  $Date: 2007/09/05$
# \author Shinji Kurihara
#
# Copyright (C) 2006
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
sys.path.insert(1,"../")

import unittest

from OutPortProvider import *
import OpenRTM_aist


class TestOutPortProvider(unittest.TestCase):
  def setUp(self):
    self._opp = OutPortProvider()
    return

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_publishInterfaceProfile(self):
    properties = []
    self._opp.setInterfaceType("corba_cdr")
    self._opp.publishInterfaceProfile(properties)
    self.assertEqual("corba_cdr", 
         OpenRTM_aist.NVUtil.toString(properties,"dataport.interface_type"))
    return

  def test_publishInterface(self):
    properties = []
    self._opp.setInterfaceType("corba_cdr")
    OpenRTM_aist.CORBA_SeqUtil.push_back(properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                         "corba_cdr"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                         "push"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                         "flush"))
    self.assertEqual(self._opp.publishInterface(properties),True)
    return

  def test_setPortType(self):
    self._opp.setPortType("out")
    return

  def test_setDataType(self):
    self._opp.setDataType("TimedLong")
    return

  def test_setInterfaceType(self):
    self._opp.setInterfaceType("corba_cdr")
    return

  def test_setDataFlowType(self):
    self._opp.setDataFlowType("flow")
    return

  def test_setSubscriptionType(self):
    self._opp.setSubscriptionType("flush")

  
############### test #################
if __name__ == '__main__':
        unittest.main()
