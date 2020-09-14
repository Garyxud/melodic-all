#!/usr/bin/env python
# -*- Python -*-

#
# \file test_PortAdmin.py
# \brief test for RTC's Port administration class
# \date $Date: 2007/09/03 $
# \author Shinji Kurihara
#
# Copyright (C) 2006
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
sys.path.insert(1,"../")

import unittest
import OpenRTM_aist
from PortAdmin import *

import RTC, RTC__POA
import OpenRTM

from omniORB import CORBA

class PortBase(OpenRTM_aist.PortBase):
  def __init__(self):
    OpenRTM_aist.PortBase.__init__(self)

  def publishInterfaces(self, prof):
    return RTC.RTC_OK

  def subscribeInterfaces(self, prof):
    return RTC.RTC_OK

  def unsubscribeInterfaces(self, prof):
    return RTC.RTC_OK

  def activateInterfaces(self):
    print "activateInterfaces"


  def deactivateInterfaces(self):
    print "deactivateInterfaces"

    
class TestPortAdmin(unittest.TestCase):
  def setUp(self):
    self._orb = CORBA.ORB_init(sys.argv)
    self._poa = self._orb.resolve_initial_references("RootPOA")
    self._poa._get_the_POAManager().activate()

    self._pa = PortAdmin(self._orb, self._poa)

    self._pb1 = PortBase()
    self._pb2 = PortBase()

    self._pb1.setName("port0")
    self._pb2.setName("port1")

    self._pa.registerPort(self._pb1)
    self._pa.registerPort(self._pb2)
    return

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()


  def test_getPortServiceList(self):
    plist = self._pa.getPortServiceList()

    prof0 = plist[0].get_port_profile()
    self.assertEqual(prof0.name, "port0")

    prof1 = plist[1].get_port_profile()
    self.assertEqual(prof1.name, "port1")
    return
    

  def test_getPortProfileList(self):
    pprof = self._pa.getPortProfileList()
    return


  def CCCtest_getPortRef(self):
    
    getP = self._pa.getPortRef("")
    self.assertEqual(CORBA.is_nil(getP), True)

    getP = self._pa.getPortRef("port1")
    self.assertEqual(CORBA.is_nil(getP), False)
    self.assertEqual(getP.get_port_profile().name, "port1")

    getP = self._pa.getPortRef("port0")
    self.assertEqual(CORBA.is_nil(getP), False)
    self.assertEqual(getP.get_port_profile().name, "port0")
    return
    

  def CCCtest_getPort(self):
    pb = self._pa.getPort("port0")
    prof = pb.get_port_profile()
    self.assertEqual(prof.name, "port0")

    pb = self._pa.getPort("port1")
    prof = pb.get_port_profile()
    self.assertEqual(prof.name, "port1")
    return


  def CCCtest_deletePort(self):
    self._pa.deletePort(self._pb1)
    plist = self._pa.getPortServiceList()
    self.assertEqual(len(plist), 1)

    prof = plist[0].get_port_profile()
    self.assertEqual(prof.name, "port1")
    return


  def CCCtest_deletePortByName(self):
    plist = self._pa.getPortServiceList()
    self.assertEqual(len(plist), 2)

    self._pa.deletePortByName("port1")

    plist = self._pa.getPortServiceList()
    self.assertEqual(len(plist), 1)
    return
    

  def CCCtest_activatePorts(self):
    self._pa.activatePorts()
    self._pa.deactivatePorts()
    return


  def CCCtest_finalizePorts(self):
    plist = self._pa.getPortServiceList()
    self.assertEqual(len(plist), 2)

    self._pa.finalizePorts()
    
    plist = self._pa.getPortServiceList()
    self.assertEqual(len(plist), 0)
    return
    


############### test #################
if __name__ == '__main__':
        unittest.main()

