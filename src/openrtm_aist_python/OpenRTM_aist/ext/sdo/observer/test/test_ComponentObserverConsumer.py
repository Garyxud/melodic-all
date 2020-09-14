#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file test_ComponentObserverConsumer.py
# @brief test for ComponentObserverConsumer
# @date $Date$
# @author Shinji Kurihara
#
# Copyright (C) 2011
#     Noriaki Ando
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
sys.path.insert(1,"../")

import unittest
import time

from omniORB import CORBA, PortableServer
import RTC
import OpenRTM, OpenRTM__POA
import SDOPackage
import OpenRTM_aist

from ComponentObserverConsumer import *

class ComponentObserver_i(OpenRTM__POA.ComponentObserver):
  def __init__(self):
    pass

  def update_status(self, status_kind, hint):
    print "update_status: ", status_kind, ", ", hint
    return


class MockRTC(OpenRTM_aist.RTObject_impl):
  def __init__(self):
    self._orb = CORBA.ORB_init()
    self._poa = self._orb.resolve_initial_references("RootPOA")
    OpenRTM_aist.RTObject_impl.__init__(self, orb=self._orb, poa=self._poa)
    pass

##
# @if jp
# @else
# @endif
#
class TestComponentObserverConsumer(unittest.TestCase):
  """
  """

  def setUp(self):
    self._orb = CORBA.ORB_init(sys.argv)
    self._poa = self._orb.resolve_initial_references("RootPOA")
    self._poa._get_the_POAManager().activate()
    self.coc = ComponentObserverConsumer()
    servant_ = ComponentObserver_i()
    mgr_ = OpenRTM_aist.Manager.instance()
    oid_ = mgr_.getPOA().servant_to_id(servant_)
    self._provider = mgr_.getPOA().id_to_reference(oid_)
    self._mock = MockRTC()
    self._properties = [OpenRTM_aist.NVUtil.newNV("heartbeat.enable","YES"),
                        OpenRTM_aist.NVUtil.newNV("heartbeat.interval","0.1"),
                        OpenRTM_aist.NVUtil.newNV("observed_status","ALL")]
    self._sprof = SDOPackage.ServiceProfile("test_id", "interface_type",
                                            self._properties, self._provider)
    self.coc.init(self._mock, self._sprof)
    return

  def tearDown(self):
    self.coc.finalize()
    del self.coc
    self._mock.exit()
    time.sleep(0.1)
    OpenRTM_aist.Manager.instance().shutdownManager()
    return


  def test_reinit(self):
    self.assertEqual(self.coc.reinit(self._sprof), True)
    return


  def test_getProfile(self):
    self.coc.getProfile()
    return


  def test_finalize(self):
    self.coc.finalize()
    return


  def test_updateStatus(self):
    self.coc.updateStatus(OpenRTM.COMPONENT_PROFILE, "update Component profile")
    return


  def test_toString(self):
    self.assertEqual("COMPONENT_PROFILE",self.coc.toString(OpenRTM.COMPONENT_PROFILE))
    self.assertEqual("RTC_STATUS",self.coc.toString(OpenRTM.RTC_STATUS))
    self.assertEqual("EC_STATUS",self.coc.toString(OpenRTM.EC_STATUS))
    self.assertEqual("PORT_PROFILE",self.coc.toString(OpenRTM.PORT_PROFILE))
    self.assertEqual("CONFIGURATION",self.coc.toString(OpenRTM.CONFIGURATION))
    self.assertEqual("HEARTBEAT",self.coc.toString(OpenRTM.HEARTBEAT))
    return


  def test_setListeners(self):
    prop = OpenRTM_aist.Properties()
    prop.setProperty("observed_status", 
                     "component_profile, rtc_status, port_profile, \
                      ec_status, port_profile , configuration")
    self.coc.setListeners(prop)
    return


  def setfunc(self):
    print "setfunc"
    return

  def unsetfunc(self):
    print "unsetfunc"
    return

  def test_switchListeners(self):
    self.coc.switchListeners(True, [True], 0, self.setfunc, self. unsetfunc)
    self.coc.switchListeners(True, [False], 0, self.setfunc, self. unsetfunc)
    self.coc.switchListeners(False, [True], 0, self.setfunc, self. unsetfunc)
    self.coc.switchListeners(False, [False], 0, self.setfunc, self. unsetfunc)
    return


  def test_heartbeat(self):
    self.coc.heartbeat()
    return


  def test_setHeartbeat(self):
    prop = OpenRTM_aist.Properties()
    prop.setProperty("heartbeat.enable","NO")
    prop.setProperty("heartbeat.interval","1.0")
    self.coc.setHeartbeat(prop)
    prop = OpenRTM_aist.Properties()
    prop.setProperty("heartbeat.enable","YES")
    prop.setProperty("heartbeat.interval","0.01")
    self.coc.setHeartbeat(prop)
    prop = OpenRTM_aist.Properties()
    prop.setProperty("heartbeat.enable","YES")
    self.coc.setHeartbeat(prop)
    return


  def test_unsetHeartbeat(self):
    self.coc.unsetHeartbeat()
    return


  def test_setComponentStatusListeners(self):
    self.coc.setComponentStatusListeners()
    return

  
  def test_unsetComponentStatusListeners(self):
    self.coc.unsetComponentStatusListeners()
    return


  def test_setPortProfileListeners(self):
    self.coc.setPortProfileListeners()
    return


  def test_unsetPortProfileListeners(self):
    self.coc.unsetPortProfileListeners()
    return


  def test_setExecutionContextListeners(self):
    self.coc.setExecutionContextListeners()
    return


  def test_unsetExecutionContextListeners(self):
    self.coc.unsetExecutionContextListeners()
    return


  def test_setConfigurationListeners(self):
    self.coc.setConfigurationListeners()
    return


  def test_unsetConfigurationListeners(self):
    self.coc.unsetConfigurationListeners()
    return


############### test #################
if __name__ == '__main__':
        unittest.main()
