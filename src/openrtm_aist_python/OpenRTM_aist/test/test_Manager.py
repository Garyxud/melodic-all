#!/usr/bin/env python
# -*- coding: euc-jp -*-

#
# \file test_Manager.py
# \brief test for RTComponent manager class
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

import sys
sys.path.insert(1,"../")

import unittest
import time

#from Manager import *
import OpenRTM_aist
import RTC, RTC__POA

testcomp_spec = ["implementation_id", "TestComp",
                 "type_name",         "TestComp",
                 "description",       "Test example component",
                 "version",           "1.0",
                 "vendor",            "Shinji Kurihara, AIST",
                 "category",          "example",
                 "activity_type",     "DataFlowComponent",
                 "max_instance",      "10",
                 "language",          "C++",
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

class TestComp(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
    return

    
def TestCompInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
  manager.registerFactory(profile,
                          TestComp,
                          OpenRTM_aist.Delete)
  
#def TestCompInit(manager):
def MyModuleInit(manager):
  TestCompInit(manager)
  com = manager.createComponent("TestComp")

def TestEcInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
  manager.registerECFactory("Art",
          TestComp,
          OpenRTM_aist.Delete)
  manager.createComponent("TestEc")


class TestManager(unittest.TestCase):

  def setUp(self):
    global manager
    #manager = OpenRTM_aist.Manager.init(sys.argv)
    #self.manager = manager
    self.manager = OpenRTM_aist.Manager.init(sys.argv)

  def tearDown(self):
    self.manager.shutdownManager()
    time.sleep(0.1)

###  def COMMENTtest_terminate(self):
###    self.manager.activateManager()
###    self.manager.runManager(True)
###    self.manager.terminate()
###    return
###
###  def COMMENTtest_shutdown(self):
###    self.manager.activateManager()
###    self.manager.runManager(True)
###    time.sleep(0.1)
###    self.manager.shutdown()
###    #self.manager.runManager()
###
###
  def test_init(self):
    #self.assertEqual(self.manager,OpenRTM_aist.Manager.init(sys.argv))
    return

  def test_instance(self):
    self.assertEqual(OpenRTM_aist.Manager.instance(), self.manager)
    return


  def test_load_unload(self):
    self.manager.load("hoge", "echo")
    self.manager.unload("hoge")
    self.manager.unloadAll()
    self.manager.load("hoge", "echo")
    self.assertEqual(len(self.manager.getLoadedModules()), 1)
    #self.assertEqual(len(self.manager.getLoadableModules()), 1)
    return

  def test_getLoadedModules(self):
    self.manager.activateManager()
    self.assertEqual(self.manager.getLoadedModules(),[])
    return

  def test_getLoadableModules(self):
    self.manager.activateManager()
    # hoge.py is loadable module.
    #self.assertNotEqual(self.manager.getLoadableModules(),[])
    return

  def test_registerFactory(self):
    profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
    self.assertEqual(self.manager.registerFactory(profile,
                                                  TestComp,
                                                  OpenRTM_aist.Delete),
                     True)
    return

  def test_getFactoryProfiles(self):
    self.manager.getFactoryProfiles()
    return

  def test_registerECFactory(self):
    profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
    self.assertEqual(self.manager.registerECFactory("Art",
                                                    TestComp,
                                                    OpenRTM_aist.Delete),
                     True)
    return

  def test_getModulesFactories(self):
    self.assertEqual(self.manager.getModulesFactories()[0],'PeriodicECSharedComposite')
    return

  def test_createComponent(self):
    self.manager.activateManager()
    profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
    self.manager.registerFactory(profile,
                                 TestComp,
                                 OpenRTM_aist.Delete)
    com = self.manager.createComponent("TestComp")
    self.assertNotEqual(com,None)
    #self.assertEqual(self.manager.getComponent("TestComp0"),self.manager.getComponents()[0])
    #self.manager.deleteComponent("TestComp0")
    self.manager.shutdownComponents()
    time.sleep(0.1)
    return

  def test_getORB(self):
    self.manager.getORB()
    return

  def test_getPOA(self):
    self.manager.getPOA()
    return

  def test_getPOAManager(self):
    self.manager.getPOAManager()
    return


  def test_registerComponent(self):
    self.manager.activateManager()
    profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
    self.manager.registerFactory(profile,
                                 TestComp,
                                 OpenRTM_aist.Delete)
    com = self.manager.createComponent("TestComp")
    self.assertEqual(self.manager.registerComponent(com),True)
    self.manager.shutdownComponents()
    return
###
###  def COMMENTtest_unregisterComponent(self):
###    self.manager.activateManager()
###    profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
###    self.manager.registerFactory(profile,
###               TestComp,
###               OpenRTM_aist.Delete)
###    com = self.manager.createComponent("TestComp")
###    self.assertEqual(self.manager.unregisterComponent(com),True)
###    self.manager.shutdownComponents()
###    return
###
  def test_createContext(self):
    ec_args = "PeriodicExecutionContext?rate="
    self.manager.activateManager()
    ec = self.manager.createContext(ec_args)
    self.assertNotEqual(ec,None)
    ec.stop()
    self.manager.getPOA().deactivate_object(self.manager.getPOA().servant_to_id(ec))
    del ec
    return
###
###
###
###  def test_shutdownManager(self):
###    return
###
###  def test_shutdownOnNoRtcs(self):
###    return
###
###    
###  def test_createORBEndpoints(self):
###    self.manager.activateManager()
###    l=[]
###    self.manager.createORBEndpoints(l)
###    return
###
  def test_createORBEndpointOption(self):
    self.manager.activateManager()
    self.manager.createORBEndpointOption("",[])
    return
###
###
###
###  def test_shutdownORB(self):
###    return
###
###  def test_shutdownComponents(self):
###    return
###
###  
###
  def test_cleanupComponent(self):
    self.manager.activateManager()
    profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
    self.manager.registerFactory(profile,
                                 TestComp,
                                 OpenRTM_aist.Delete)
    com = self.manager.createComponent("TestComp")
    self.assertEqual(self.manager.registerComponent(com),True)
    self.manager.cleanupComponent(com)
    com.exit()
    self.manager.shutdownComponents()
    return


  def test_notifyFinalized(self):
    self.manager.activateManager()
    profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
    self.manager.registerFactory(profile,
                                 TestComp,
                                 OpenRTM_aist.Delete)
    com = self.manager.createComponent("TestComp")
    #self.manager.notifyFinalized(com)
    #self.manager.cleanupComponents()
    self.manager.shutdownComponents()
    return


  def test_initManager(self):
    self.manager.initManager(sys.argv)
    #self.manager.shutdownManager()
    return

  def test_initLogger(self):
    self.manager.initLogger()
    #self.manager.shutdownLogger()
    return

  def test_initORB(self):
    #self.assertEqual(self.manager.initORB(),True)
    #self.manager.shutdownORB()
    return


  def test_initNaming(self):
    self.assertEqual(self.manager.initNaming(),True)
    #self.manager.shutdownNaming()
    return


  def test_initExecContext(self):
    self.assertEqual(self.manager.initExecContext(),True)
    return

  def test_initComposite(self):
    self.assertEqual(self.manager.initComposite(),True)
    return


  def test_initFactories(self):
    self.assertEqual(self.manager.initFactories(),True)
    return

  def test_initManagerServant(self):
    self.assertEqual(self.manager.initManagerServant(), True)
    return


  def test_procComponentArgs(self):
    comp_id = OpenRTM_aist.Properties()
    comp_conf = OpenRTM_aist.Properties()

    self.assertEqual(self.manager.procComponentArgs("TestComp?instance_name=test&exported_ports=ConsoleIn0.out,ConsoleOut0.in",comp_id,comp_conf),True)
    self.assertEqual(comp_id.getProperty("implementation_id"),"TestComp")
    self.assertEqual(comp_conf.getProperty("instance_name"),"test")
    self.assertEqual(comp_conf.getProperty("exported_ports"),"ConsoleIn0.out,ConsoleOut0.in")
    return

  def test_procContextArgs(self):
    ec_id = [""]
    ec_conf = OpenRTM_aist.Properties()
    self.assertEqual(self.manager.procContextArgs("PeriodicExecutionContext?rate=1000",ec_id,ec_conf),True)
    self.assertEqual(ec_id[0],"PeriodicExecutionContext")
    self.assertEqual(ec_conf.getProperty("rate"),"1000")
    return

  def test_configureComponent(self):
    self.manager.activateManager()
    profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
    self.manager.registerFactory(profile,
                                 TestComp,
                                 OpenRTM_aist.Delete)
    com = self.manager.createComponent("TestComp")
    prop = OpenRTM_aist.Properties()
    self.manager.configureComponent(com,prop)
    self.manager.cleanupComponent(com)
    com.exit()
    self.manager.shutdownComponents()
    return

  def test_formatString(self):
    profile = OpenRTM_aist.Properties(defaults_str=testcomp_spec)
    self.assertEqual(self.manager.formatString("rtc.log",profile),"rtc.log")
    print self.manager.formatString("$(PWD)",profile)
    print self.manager.formatString("%n.%t.%m.%v.%V.%c.%h.%M.%p",profile)
    return

  def test_getLogbuf(self):
    self.manager.getLogbuf()
    return

  def test_getConfig(self):
    self.manager.getConfig()
    return




############### test #################
if __name__ == '__main__':
        unittest.main()
