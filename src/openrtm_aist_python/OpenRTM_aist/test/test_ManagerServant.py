#!/usr/bin/env python
# -*- coding: euc-jp -*-

#
# \file test_ManagerServant.py
# \brief test for ManagerServant class
# \date $Date: $
# \author Shinji Kurihara
#
# Copyright (C) 2006
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys,time
sys.path.insert(1,"../RTM_IDL")
sys.path.insert(1,"../")

import unittest
from omniORB import CORBA

#from Manager import *
import OpenRTM_aist
import RTC, RTC__POA
import RTM, RTM__POA

configsample_spec = ["implementation_id", "TestComp",
         "type_name",         "TestComp",
         "description",       "Test example component",
         "version",           "1.0",
         "vendor",            "Shinji Kurihara, AIST",
         "category",          "example",
         "activity_type",     "DataFlowComponent",
         "max_instance",      "10",
         "language",          "Python",
         "lang_type",         "compile",
         # Configuration variables
         "conf.default.int_param0", "0",
         "conf.default.int_param1", "1",
         "conf.default.double_param0", "0.11",
         "conf.default.double_param1", "9.9",
         "conf.default.str_param0", "hoge",
         "conf.default.str_param1", "dara",
         "conf.default.vector_param0", "0.0,1.0,2.0,3.0,4.0",
         ""]

com = None

class TestComp(OpenRTM_aist.DataFlowComponentBase):
  def __init_(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

    
def TestCompInit(manager):
  print "TestCompInit"
  global com
  profile = OpenRTM_aist.Properties(defaults_str=configsample_spec)
  manager.registerFactory(profile,
        TestComp,
        OpenRTM_aist.Delete)
  com = manager.createComponent("TestComp")

def TestEcInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=configsample_spec)
  print manager.registerECFactory("Art",
          TestComp,
          OpenRTM_aist.Delete)
  manager.createComponent("TestEc")


class TestManagerServant(unittest.TestCase):

  def setUp(self):
    self.managerservant = OpenRTM_aist.ManagerServant()

  def tearDown(self):
    self.managerservant.__del__()
    del self.managerservant
    #OpenRTM_aist.Manager.instance().shutdownManager()
    OpenRTM_aist.Manager.instance().shutdown()
    time.sleep(0.1)
    return

  """
  def test_terminate(self):
    self.managerservant.terminate()

  def test_shutdown(self):
    self.managerservant.runManager(True)
    import time
    time.sleep(0.1)
    self.managerservant.shutdown()
    #self.managerservant.runManager()
  """

  def test_load_unload(self):
    self.managerservant.load_module("hoge", "echo")
    self.assertNotEqual(len(self.managerservant.get_loaded_modules()), 0)
    self.assertNotEqual(len(self.managerservant.get_loadable_modules()), 0)
    self.managerservant.unload_module("hoge")
    return

  def test_get_loaded_modules(self):
    self.assertEqual(self.managerservant.get_loaded_modules(),[])
    return

  def test_get_loadable_modules(self):
    self.assertNotEqual(self.managerservant.get_loadable_modules(),[])
    return


  def test_get_factory_profiles(self):
    self.managerservant.get_factory_profiles()
    return

  def test_create_component(self):
    mgr=OpenRTM_aist.Manager.init(sys.argv)
    mgr.activateManager()
    self.managerservant.load_module("test_ManagerServant", "TestCompInit")
    com = self.managerservant.create_component("TestComp")
    self.managerservant.unload_module("test_ManagerServant")
    self.assertNotEqual(com,None)
    self.managerservant.delete_component("TestComp")
    mgr.shutdownComponents()
    return


  def test_get_components(self):
    self.assertEqual(self.managerservant.get_components(),[])
    return


  def test_get_component_profiles(self):
    self.assertEqual(self.managerservant.get_component_profiles(),[])
    return


  def test_get_profile(self):
    self.managerservant.get_profile()
    return
    
  def test_get_configuration(self):
    self.managerservant.get_configuration()
    return

  def test_set_configuration(self):
    self.assertEqual(self.managerservant.set_configuration("test_name", "test_value"),RTC.RTC_OK)
    return


  def test_get_service(self):
    self.assertEqual(self.managerservant.get_service("test"),CORBA.Object._nil)
    return


  def test_getObjRef(self):
    if not self.managerservant.createINSManager():
      poa=OpenRTM_aist.Manager.instance().getORB().resolve_initial_references("omniINSPOA")
      poa.deactivate_object(OpenRTM_aist.Manager.instance().getConfig().getProperty("manager.name"))
      self.assertEqual(self.managerservant.createINSManager(),True)
    else:
      self.assertEqual(self.managerservant.createINSManager(),False)
    self.assertNotEqual(self.managerservant.getObjRef(),CORBA.Object._nil)
    return

  def test_is_master(self):
    self.assertEqual(self.managerservant.is_master(),False)
    return

  def test_get_master_managers(self):
    self.assertEqual(len(self.managerservant.get_master_managers()),0)
    return

  def test_master_manager(self):
    if not self.managerservant.createINSManager():
      poa=OpenRTM_aist.Manager.instance().getORB().resolve_initial_references("omniINSPOA")
      poa.deactivate_object(OpenRTM_aist.Manager.instance().getConfig().getProperty("manager.name"))
      self.assertEqual(self.managerservant.createINSManager(),True)
    else:
      self.assertEqual(self.managerservant.createINSManager(),False)
    self.assertEqual(len(self.managerservant.get_master_managers()),0)
    host_port = "localhost:2810"
    owner = self.managerservant.findManager(host_port)
    self.assertEqual(self.managerservant.add_master_manager(owner),RTC.RTC_OK)
    self.assertEqual(len(self.managerservant.get_master_managers()),1)
    self.assertEqual(self.managerservant.remove_master_manager(owner),RTC.RTC_OK)
    self.assertEqual(len(self.managerservant.get_master_managers()),0)
    
    return


  def test_slave_managers(self):
    if not self.managerservant.createINSManager():
      poa=OpenRTM_aist.Manager.instance().getORB().resolve_initial_references("omniINSPOA")
      poa.deactivate_object(OpenRTM_aist.Manager.instance().getConfig().getProperty("manager.name"))
      self.assertEqual(self.managerservant.createINSManager(),True)
    else:
      self.assertEqual(self.managerservant.createINSManager(),False)
    self.assertEqual(len(self.managerservant.get_slave_managers()),0)
    host_port = "localhost:2810"
    owner = self.managerservant.findManager(host_port)
    self.assertEqual(self.managerservant.add_slave_manager(owner),RTC.RTC_OK)
    self.assertEqual(len(self.managerservant.get_slave_managers()),1)
    self.assertEqual(self.managerservant.remove_slave_manager(owner),RTC.RTC_OK)
    self.assertEqual(len(self.managerservant.get_slave_managers()),0)
    return

    
"""
  def test_forck(self):
    self.assertEqual(self.managerservant.fork(),RTC.RTC_OK)
    return

  def test_shutdown(self):
    #self.assertEqual(self.managerservant.shutdown(),RTC.RTC_OK)
    return

  def test_restart(self):
    self.assertEqual(self.managerservant.restart(),RTC.RTC_OK)
    return

"""



############### test #################
if __name__ == '__main__':
        unittest.main()
