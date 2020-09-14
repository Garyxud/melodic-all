#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file test_SdoServiceAdmin.py
# @brief test for SdoServiceAdmin class
# @date $Date$
# @author Shinji Kurihara
#
# Copyright (C) 2011
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import sys
sys.path.insert(1, "../")

import unittest
from SdoServiceAdmin import *
import OpenRTM_aist
import SDOPackage,SDOPackage__POA
from omniORB import CORBA, PortableServer
from omniORB import any


class MockRTC(OpenRTM_aist.RTObject_impl):
  def __init__(self):
    self._orb = CORBA.ORB_init()
    self._poa = self._orb.resolve_initial_references("RootPOA")
    OpenRTM_aist.RTObject_impl.__init__(self, orb=self._orb, poa=self._poa)
    pass


class SDOServiceMock(SDOPackage__POA.SDOService):
  def __init__(self):
    pass

class MockSdoServiceConsumer(OpenRTM_aist.SdoServiceConsumerBase):
  """
  """

  def __init__(self):
    OpenRTM_aist.SdoServiceConsumerBase.__init__(self)
    return

  def reinit(self, profile):
    pass

  def getProfile(self):
    any_val = any.to_any("3.14159")
    nv = SDOPackage.NameValue("PROPERTIES NAME 0", any_val)
    sprof = SDOPackage.ServiceProfile("test id","INTERFACE_TYPE",[nv],SDOServiceMock())
    return sprof


class TestListener(unittest.TestCase):
  def setUp(self):
    return


  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  
  def test_addSdoServiceConsumerFactory(self):
    sdoadmin = SdoServiceAdmin(MockRTC())
    self.assertEqual(sdoadmin.addSdoServiceConsumerFactory(),False)
    return


  def test_removeSdoServiceConsumerFactory(self):
    sdoadmin = SdoServiceAdmin(MockRTC())
    self.assertEqual(sdoadmin.removeSdoServiceConsumerFactory(),False)
    return


  def test_addSdoServiceConsumer(self):
    any_val = any.to_any("3.14159")
    nv = SDOPackage.NameValue("PROPERTIES NAME 0", any_val)
    sprof = SDOPackage.ServiceProfile("ID 0","INTERFACE_TYPE",[nv],SDOServiceMock())
    sdoadmin = SdoServiceAdmin(MockRTC())
    self.assertEqual(sdoadmin.addSdoServiceConsumer(sprof),False)
    return


  def test_removeSdoServiceConsumer(self):
    any_val = any.to_any("3.14159")
    nv = SDOPackage.NameValue("PROPERTIES NAME 0", any_val)
    sprof = SDOPackage.ServiceProfile("test id","INTERFACE_TYPE",[nv],SDOServiceMock())
    sdoadmin = SdoServiceAdmin(MockRTC())
    self.assertEqual(sdoadmin.addSdoServiceConsumer(sprof),False)
    sdoadmin._consumers.append(MockSdoServiceConsumer())
    self.assertEqual(sdoadmin.removeSdoServiceConsumer("test id"),True)
    self.assertEqual(sdoadmin.addSdoServiceConsumer(sprof),False)
    sdoadmin._consumers.append(MockSdoServiceConsumer())
    self.assertEqual(sdoadmin.removeSdoServiceConsumer("test id2"),False)
    return


  def test_isAllowedConsumerType(self):
    sdoadmin = SdoServiceAdmin(MockRTC())
    any_val = any.to_any("3.14159")
    nv = SDOPackage.NameValue("PROPERTIES NAME 0", any_val)
    sprof = SDOPackage.ServiceProfile("test id","INTERFACE_TYPE",[nv],SDOServiceMock())
    self.assertEqual(sdoadmin.isAllowedConsumerType(sprof),True)
    sdoadmin._allConsumerAllowed = False
    self.assertEqual(sdoadmin.isAllowedConsumerType(sprof),False)
    sdoadmin._consumerTypes = ["type0","type1","type2","INTERFACE_TYPE"]
    self.assertEqual(sdoadmin.isAllowedConsumerType(sprof),True)
    sdoadmin._consumerTypes = ["type0","type1","type2"]
    self.assertEqual(sdoadmin.isAllowedConsumerType(sprof),False)
    return


  def test_isExistingConsumerType(self):
    factory = OpenRTM_aist.SdoServiceConsumerFactory.instance()
    factory.addFactory("test_factory",OpenRTM_aist.SdoServiceConsumerBase,OpenRTM_aist.Delete)
    sdoadmin = SdoServiceAdmin(MockRTC())
    any_val = any.to_any("3.14159")
    nv = SDOPackage.NameValue("PROPERTIES NAME 0", any_val)
    sprof = SDOPackage.ServiceProfile("test id","INTERFACE_TYPE",[nv],SDOServiceMock())
    self.assertEqual(sdoadmin.isExistingConsumerType(sprof),False)
    factory.addFactory("INTERFACE_TYPE",OpenRTM_aist.SdoServiceConsumerBase,OpenRTM_aist.Delete)
    self.assertEqual(sdoadmin.isExistingConsumerType(sprof),True)
    return


  def test_getUUID(self):
    sdoadmin = SdoServiceAdmin(MockRTC())
    self.assertNotEqual(sdoadmin.getUUID(),"")
    self.assertNotEqual(sdoadmin.getUUID(),None)
    return


############### test #################
if __name__ == '__main__':
  unittest.main()
