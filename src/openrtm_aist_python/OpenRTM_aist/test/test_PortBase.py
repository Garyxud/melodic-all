#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_PortBase.py
#  \brief test for RTC's Port base class
#  \date $Date: 2007/09/18 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 

import sys
sys.path.insert(1,"../")

import unittest
from PortBase import *

import CORBA
import OpenRTM_aist
import RTC, RTC__POA


class RTObjMock(OpenRTM_aist.DataFlowComponentBase):
  def __init_(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

class OutPortObj(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, OpenRTM_aist.Manager.instance())
    self.addOutPort("out",OpenRTM_aist.OutPort("out", RTC.TimedLong(RTC.Time(0,0),0)))


class InPortObj(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, OpenRTM_aist.Manager.instance())
    self.addInPort("in",OpenRTM_aist.InPort("in", RTC.TimedLong(RTC.Time(0,0),0)))


class TestPortBase(unittest.TestCase):
  def setUp(self):
    OpenRTM_aist.Manager.init(sys.argv)
    OpenRTM_aist.Manager.instance().activateManager()
    self._pb = PortBase()
    return

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_get_port_profile(self):
    self._pb.setName("test_connect")
    prof = self._pb.get_port_profile()
    self.assertEqual(prof.name,"test_connect")
    return

  def test_getPortProfile(self):
    self._pb.setName("test_connect")
    prof = self._pb.getPortProfile()
    self.assertEqual(prof.name,"test_connect")
    

  def test_get_connector_profiles(self):
    OpenRTM_aist.Manager.init(sys.argv)
    nvlist = [OpenRTM_aist.NVUtil.newNV("dataport.interface_type","corba_cdr"),
        OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type","push"),
        OpenRTM_aist.NVUtil.newNV("dataport.subscription_type","flush")]

    outp = OutPortObj()
    outp = outp.get_ports()

    inp  = InPortObj()
    inp = inp.get_ports()
    prof = RTC.ConnectorProfile("connector0",
              "connectorid_0",
              [outp[0]._narrow(RTC.PortService),inp[0]._narrow(RTC.PortService)],
              nvlist)
    ret, prof = self._pb.connect(prof)
    cntlist = outp[0].get_connector_profiles()
    self.assertEqual(cntlist[0].name,"connector0")
    cntprofile = outp[0].get_connector_profile("connectorid_0")
    self.assertEqual(cntprofile.connector_id,"connectorid_0")
    #ret,prof = self._pb.notify_connect(prof)
    return



  def test_connect_disconnect(self):
    nvlist = [OpenRTM_aist.NVUtil.newNV("dataport.interface_type","corba_cdr"),
        OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type","push"),
        OpenRTM_aist.NVUtil.newNV("dataport.subscription_type","flush")]

    outp = OutPortObj().get_ports()
    inp  = InPortObj().get_ports()
    prof = RTC.ConnectorProfile("connector0","connector_id1",[inp[0],outp[0]],nvlist)
    self._pb.connect(prof)
    cntprofile = outp[0].get_connector_profile("connector_id1")
    self.assertEqual(cntprofile.connector_id,"connector_id1")
    outp[0].disconnect("connector_id1")
    cntprofile = outp[0].get_connector_profile("connector_id1")
    self.assertNotEqual(cntprofile.connector_id,"connector_id1")
    self._pb.connect(prof)
    outp[0].disconnect_all()
    self.assertNotEqual(cntprofile.connector_id,"connector_id1")
    return



  def test_getProfile(self):
    outp = OutPortObj().get_ports()
    self._pb.setName("test")
    self._pb.setPortRef(outp[0])
    rtobj = RTObjMock(OpenRTM_aist.Manager.instance())
    self._pb.setOwner(rtobj)
    prof = self._pb.getProfile()
    self.assertEqual(prof.name,".test")
    self.assertEqual(prof.port_ref,outp[0])
    self.assertEqual(prof.port_ref,outp[0])
    self.assertEqual(prof.port_ref,self._pb.getPortRef())
    return


  def test_getUUID(self):
    self._pb.getUUID()
    return


  def test_updateConnectorProfile(self):
    nvlist = [OpenRTM_aist.NVUtil.newNV("dataport.interface_type","corba_cdr"),
        OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type","push"),
        OpenRTM_aist.NVUtil.newNV("dataport.subscription_type","flush")]

    outp = OutPortObj().get_ports()
    inp  = InPortObj().get_ports()
    prof = RTC.ConnectorProfile("connector0","connector_id1",[inp[0],outp[0]],nvlist)
    self._pb.updateConnectorProfile(prof)
    cntlist = self._pb.get_connector_profiles()
    self.assertEqual(cntlist[0].name,"connector0")
    return



  def test_eraseConnectorProfile(self):
    nvlist = [OpenRTM_aist.NVUtil.newNV("dataport.interface_type","corba_cdr"),
        OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type","push"),
        OpenRTM_aist.NVUtil.newNV("dataport.subscription_type","flush")]

    outp = OutPortObj().get_ports()
    inp  = InPortObj().get_ports()
    prof = RTC.ConnectorProfile("connector0","connector_id1",[inp[0],outp[0]],nvlist)
    self._pb.connect(prof)
    #self.assertEqual(self._pb.eraseConnectorProfile("connector_id1"),True)
    return 



  def test_append_deleteInterface(self):
    self.assertEqual(self._pb.appendInterface("name", "type", RTC.PROVIDED), True)
    piprof = RTC.PortInterfaceProfile("name","type",RTC.PROVIDED)
    self._pb._profile=RTC.PortProfile("name",[piprof],RTC.PortService._nil,[],RTC.RTObject._nil,[])
    self.assertEqual(self._pb.appendInterface("name", "type", RTC.PROVIDED), False)
    self.assertEqual(self._pb.deleteInterface("name", RTC.PROVIDED), True)
    self.assertEqual(self._pb.deleteInterface("name", RTC.PROVIDED), False)
    return
    

  def test_add_appendProperty(self):
    self._pb.addProperty("name1","val1")
    prof = self._pb.getPortProfile()
    self.assertEqual(prof.properties[0].name, "name1")
    self._pb.appendProperty("name2","val2")
    prof = self._pb.getPortProfile()
    self.assertEqual(prof.properties[1].name, "name2")


############### test #################
if __name__ == '__main__':
        unittest.main()
