#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_OutPortBase.py
#  \brief test for OutPortBase base class
#  \date $Date: 2007/09/19 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2003-2006
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.

import sys
sys.path.insert(1,"../")

#from omniORB import any

import unittest
from OutPortBase import *

import OpenRTM_aist
import RTC

counter = 0

class PublisherTest:
  def __init__(self):
    global counter
    self.counter = counter
    counter+=1

  def update(self):
    print "update",self.counter
    
class ConnectorMock:
  def __init__(self, name="name", id="id", ports=None, properties=None):
    self._name       = name
    self._id         = id
    self._ports      = ports
    self._properties = properties
    return

  def profile(self):
    return OpenRTM_aist.ConnectorInfo(self._name,self._id,self._ports,self._properties)

  def id(self):
    return self._id

  def name(self):
    return self._name

  def activate(self):
    pass

  def deactivate(self):
    pass


class ConsumerMock:
  def init(self,prop):
    return

class TestOutPortBase(unittest.TestCase):
  def setUp(self):
    OpenRTM_aist.Manager.init(sys.argv)
    self._opb = OutPortBase("test","TimedLong")
    self._opb.init(OpenRTM_aist.Properties())
    return

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_getName(self):
    self.assertEqual(self._opb.getName(),"unknown.test")
    return

  def test_connectors(self):
    self.assertEqual(self._opb.connectors(),[])
    return

  def test_getConnectorProfiles(self):
    self.assertEqual(self._opb.getConnectorProfiles(),[])
    self._opb._connectors = [ConnectorMock()]
    cprof = self._opb.getConnectorProfiles()[0]
    self.assertEqual(cprof.name,"name")
    return

  def test_getConnectorIds(self):
    self.assertEqual(self._opb.getConnectorIds(),[])
    self._opb._connectors = [ConnectorMock()]
    cprof = self._opb.getConnectorIds()[0]
    self.assertEqual(cprof,"id")
    return
    
  def test_getConnectorProfileById(self):
    self.assertEqual(self._opb.getConnectorProfileById("id",[]),False)
    self._opb._connectors = [ConnectorMock()]
    prof = [None]
    self.assertEqual(self._opb.getConnectorProfileById("id",prof),True)
    self.assertEqual(prof[0].id,"id")
    return

  def test_getConnectorProfileByName(self):
    self.assertEqual(self._opb.getConnectorProfileByName("name",[]),False)
    self._opb._connectors = [ConnectorMock()]
    prof = [None]
    self.assertEqual(self._opb.getConnectorProfileByName("name",prof),True)
    self.assertEqual(prof[0].name,"name")
    return

  def test_activateInterfaces(self):
    self._opb._connectors = [ConnectorMock()]
    self._opb.activateInterfaces()
    self._opb.deactivateInterfaces()
    return

  def test_publishInterfaces(self):
    cprof = RTC.ConnectorProfile("","",[],[])
    self.assertEqual(self._opb.publishInterfaces(cprof),RTC.BAD_PARAMETER)
    cprof = RTC.ConnectorProfile("connecto0","",[],[])
    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                         "corba_cdr"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                         "push"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                         "flush"))

    self.assertEqual(self._opb.publishInterfaces(cprof),RTC.RTC_OK)
    return

  def test_subscribeInterfaces(self):
    cprof = RTC.ConnectorProfile("","",[],[])
    self.assertEqual(self._opb.subscribeInterfaces(cprof),RTC.BAD_PARAMETER)
    cprof = RTC.ConnectorProfile("connecto0","",[],[])
    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                         "corba_cdr"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                         "push"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                         "flush"))

    #self.assertEqual(self._opb.subscribeInterfaces(cprof),RTC.RTC_OK)
    return
    

  def test_unsubscribeInterfaces(self):
    self._opb._connectors = [ConnectorMock()]
    cprof = RTC.ConnectorProfile("connecto0","",[],[])
    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                         "corba_cdr"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                         "push"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                         "flush"))
    self._opb.unsubscribeInterfaces(cprof)
    return

  def test_initProviders(self):
    self._opb.initProviders()
    return

  def test_initConsumers(self):
    self._opb.initConsumers()
    return

  def test_createProvider(self):
    self._opb._connectors = [ConnectorMock()]
    cprof = RTC.ConnectorProfile("connecto0","",[],[])
    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                         "corba_cdr"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                         "push"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                         "flush"))
    prop = OpenRTM_aist.Properties()
    prop.setProperty("interface_type","corba_cdr")
    prop.setProperty("dataflow_type","pull")
    prop.setProperty("subscription_type","flush")
    self.assertNotEqual(self._opb.createProvider(cprof,prop),0)

    return

  def test_createConsumer(self):
    self._opb._connectors = [ConnectorMock()]
    cprof = RTC.ConnectorProfile("connecto0","",[],[])
    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                         "corba_cdr"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                         "push"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                         "flush"))
    prop = OpenRTM_aist.Properties()
    prop.setProperty("interface_type","corba_cdr")
    prop.setProperty("dataflow_type","push")
    prop.setProperty("subscription_type","flush")
    self.assertEqual(self._opb.createConsumer(cprof,prop),0)

    return

  def test_createConnector(self):
    self._opb._connectors = [ConnectorMock()]
    cprof = RTC.ConnectorProfile("connecto0","",[],[])
    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.interface_type",
                         "corba_cdr"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type",
                         "push"))

    OpenRTM_aist.CORBA_SeqUtil.push_back(cprof.properties,
                 OpenRTM_aist.NVUtil.newNV("dataport.subscription_type",
                         "flush"))
    prop = OpenRTM_aist.Properties()
    prop.setProperty("interface_type","corba_cdr")
    prop.setProperty("dataflow_type","push")
    prop.setProperty("subscription_type","flush")
    self.assertNotEqual(self._opb.createConnector(cprof,prop,consumer_=ConsumerMock()),0)

    return

  def test_getConnectorById(self):
    self.assertEqual(self._opb.getConnectorById("test"),0)
    self._opb._connectors = [ConnectorMock()]
    prof = [None]
    conn = self._opb.getConnectorById("id")
    self.assertEqual(conn.id(),"id")
    return


  def test_getConnectorByName(self):
    self.assertEqual(self._opb.getConnectorByName("test"),0)
    self._opb._connectors = [ConnectorMock()]
    prof = [None]
    conn = self._opb.getConnectorByName("name")
    self.assertEqual(conn.name(),"name")
    return


############### test #################
if __name__ == '__main__':
  unittest.main()
