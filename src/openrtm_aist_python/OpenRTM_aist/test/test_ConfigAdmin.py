#!/usr/bin/env python
# -*- coding: euc-jp -*- 

#
# \file test_ConfigAdmin.py
# \brief test for Configuration Administration classes
# \date $Date: 2007/09/04$
# \author Shinji Kurihara
#
# Copyright (C) 2007
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import sys
sys.path.insert(1,"../")

import unittest

import OpenRTM_aist
from ConfigAdmin import *

configsample_spec = ["implementation_id", "ConfigSample",
         "type_name",         "ConfigSample",
         "description",       "Configuration example component",
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

configsample_mode_spec = ["conf.default.int_param0", "10",
        "conf.default.int_param1", "11",
        "conf.default.double_param0", "0.22",
        "conf.default.double_param1", "9999.9",
        "conf.default.str_param0", "hogehoge",
        "conf.default.str_param1", "daradaradata",
        "conf.default.vector_param0", "0.1,1.1,2.1,3.1,4.1",
        ""]

configsample_add_spec = ["conf.mode0.int_param0", "10",
       "conf.mode0.int_param1", "11",
       "conf.mode0.double_param0", "0.22",
       "conf.mode0.double_param1", "9999.9",
       "conf.mode0.str_param0", "hogehoge",
       "conf.mode0.str_param1", "daradaradata",
       "conf.mode0.vector_param0", "0.1,1.1,2.1,3.1,4.1",
       ""]

class TestConfigAdmin(unittest.TestCase):
  def setUp(self):
    prop = OpenRTM_aist.Properties(defaults_str=configsample_spec)
    self._ca = ConfigAdmin(prop.getNode("conf"))

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return
  
  def CallBack(self,*args):
    #print args
    return 

  def test_bindParameter(self):
    self.int_param0    = [0]
    self.int_param1    = [0]
    self.double_param0 = [0.0]
    self.double_param1 = [0.0]
    self.str_param0    = [""]
    self.str_param1    = [""]
    self.vector_param0 = [[0.0,0.0,0.0,0.0,0.0]]

    self._ca.bindParameter("int_param0", self.int_param0, "0")
    self._ca.bindParameter("int_param1", self.int_param1, "1")
    self._ca.bindParameter("double_param0", self.double_param0, "0.11")
    self._ca.bindParameter("double_param1", self.double_param1, "9.9")
    self._ca.bindParameter("str_param0", self.str_param0, "hoge")
    self._ca.bindParameter("str_param1", self.str_param1, "dara")
    self._ca.bindParameter("vector_param0", self.vector_param0, "0.0,1.0,2.0,3.0,4.0")
    return

  def test_update(self):
    self._ca.update(config_set="default")
    self._ca.update("default","int_param0")
    self._ca.update()
    return

  def test_isExist(self):
    varName = "name"
    prop = OpenRTM_aist.Properties()
    ca = ConfigAdmin(prop)
    
    varName = "name"
    var = [0.0]
    default_value = "3.14159"

    self.assertEqual(True, ca.bindParameter(varName,var,default_value))
    self.assertEqual(3.14159,var[0])

    # バインドした変数の名称でisExist()を呼出し、真値が得られるか？
    self.assertEqual(True, ca.isExist("name"))
      
    # バインドしていない名称でisExist()を呼出し、偽値が得られるか？
    self.assertEqual(False, ca.isExist("inexist name"))
    
    return

  def test_isChanged(self):
    self.assertEqual(self._ca.isChanged(),False)
    return
  

  def test_getActiveId(self):
    self.assertEqual(self._ca.getActiveId(),"default")
    return


  def test_haveConfig(self):
    self.assertEqual(self._ca.haveConfig("default"),True)
    # Failure pattern
    # self.assertEqual(self._ca.haveConfig("int_param0"),True)
    return


  def test_isActive(self):
    self.assertEqual(self._ca.isActive(),True)
    return


  def test_getConfigurationSets(self):
    self.assertEqual(self._ca.getConfigurationSets()[0].name,"default")
    return


  def test_getConfigurationSet(self):
    self.assertEqual(self._ca.getConfigurationSet("default").name, "default")
    return

  def test_setConfigurationSetValues(self):
    prop = OpenRTM_aist.Properties(defaults_str=configsample_mode_spec)
    self.assertEqual(self._ca.setConfigurationSetValues(prop.getNode("conf.default")),True)
    return
  
  def test_getActiveConfigurationSet(self):
    self.assertEqual(self._ca.getActiveConfigurationSet().getName(),"default")
    return
  
  def test_addConfigurationSet(self):
    prop = OpenRTM_aist.Properties(defaults_str=configsample_add_spec)
    self.assertEqual(self._ca.addConfigurationSet(prop.getNode("conf.mode0")),True)
    return

  def test_removeConfigurationSet(self):
    prop = OpenRTM_aist.Properties(defaults_str=configsample_add_spec)
    self.assertEqual(self._ca.addConfigurationSet(prop.getNode("conf.mode0")),True)
    self.assertEqual(self._ca.removeConfigurationSet("mode0"),True)
    return

  
  def test_activateConfigurationSet(self):
    prop = OpenRTM_aist.Properties(defaults_str=configsample_add_spec)
    self.assertEqual(self._ca.addConfigurationSet(prop.getNode("conf.mode0")),True)
    self.assertEqual(self._ca.activateConfigurationSet("mode0"),True)
    self.assertEqual(self._ca.activateConfigurationSet("default"),True)
    self.assertEqual(self._ca.activateConfigurationSet("mode0"),True)
    # Failure pattern
    # self.assertEqual(self._ca.activateConfigurationSet("mode1"),True)
    return

  def test_setOnUpdate(self):
    self._ca.setOnUpdate(None)
    return

  def test_setOnUpdateParam(self):
    self._ca.setOnUpdateParam(None)
    return

  def test_setOnSetConfigurationSet(self):
    self._ca.setOnSetConfigurationSet(None)
    return

  def test_setOnAddConfigurationSet(self):
    self._ca.setOnAddConfigurationSet(None)
    return 


  def test_setOnRemoveConfigurationSet(self):
    self._ca.setOnRemoveConfigurationSet(None)
    return 


  def test_setOnActivateSet(self):
    self._ca.setOnActivateSet(None)
    return 

  
  def test_onUpdate(self):
    self._ca.setOnUpdate(self.CallBack)
    self._ca.onUpdate("onUpdate")
    return 


  def test_onUpdateParam(self):
    self._ca.setOnUpdateParam(self.CallBack)
    self._ca.onUpdateParam("onUpdateParam","Param")
    return 


  def test_onSetConfigurationSet(self):
    self._ca.setOnSetConfigurationSet(self.CallBack)
    self._ca.onSetConfigurationSet("onSetConfigurationSet")
    return 


  def test_onAddConfigurationSet(self):
    self._ca.setOnAddConfigurationSet(self.CallBack)
    self._ca.onAddConfigurationSet("onAddConfigurationSet")
    return 


  def test_onRemoveConfigurationSet(self):
    self._ca.setOnRemoveConfigurationSet(self.CallBack)
    self._ca.onRemoveConfigurationSet("onRemoveConfigurationSet")
    return 


  def test_onActivateSet(self):
    self._ca.setOnActivateSet(self.CallBack)
    self._ca.onActivateSet("ActivateSet")
    return 

    
  def test_addremoveConfigurationParamListener(self):
    listener = ConfigParamListenerCallback()
    self._ca.addConfigurationParamListener(OpenRTM_aist.ConfigurationParamListenerType.ON_UPDATE_CONFIG_PARAM,
                                           listener)
    self._ca.onUpdateParam("","")
    self._ca.removeConfigurationParamListener(OpenRTM_aist.ConfigurationParamListenerType.ON_UPDATE_CONFIG_PARAM,
                                              listener)
    return

  def test_addremoveConfigurationSetListener(self):
    listener = ConfigSetListenerCallback()
    self._ca.addConfigurationSetListener(OpenRTM_aist.ConfigurationSetListenerType.ON_SET_CONFIG_SET,
                                         listener)
    self._ca.onSetConfigurationSet(None)
    self._ca.removeConfigurationSetListener(OpenRTM_aist.ConfigurationSetListenerType.ON_SET_CONFIG_SET,
                                            listener)
    self._ca.addConfigurationSetListener(OpenRTM_aist.ConfigurationSetListenerType.ON_ADD_CONFIG_SET,
                                         listener)
    self._ca.onSetConfigurationSet(None)
    self._ca.removeConfigurationSetListener(OpenRTM_aist.ConfigurationSetListenerType.ON_ADD_CONFIG_SET,
                                            listener)
    return

  def test_addremoveConfigurationSetNameListener(self):
    listener = ConfigSetNameListenerCallback()
    self._ca.addConfigurationSetNameListener(OpenRTM_aist.ConfigurationSetNameListenerType.ON_UPDATE_CONFIG_SET,
                                             listener)
    self._ca.onUpdate("")
    self._ca.removeConfigurationSetNameListener(OpenRTM_aist.ConfigurationSetNameListenerType.ON_UPDATE_CONFIG_SET,
                                                listener)

    self._ca.addConfigurationSetNameListener(OpenRTM_aist.ConfigurationSetNameListenerType.ON_REMOVE_CONFIG_SET,
                                             listener)
    self._ca.onUpdate("")
    self._ca.removeConfigurationSetNameListener(OpenRTM_aist.ConfigurationSetNameListenerType.ON_REMOVE_CONFIG_SET,
                                                listener)

    self._ca.addConfigurationSetNameListener(OpenRTM_aist.ConfigurationSetNameListenerType.ON_ACTIVATE_CONFIG_SET,
                                             listener)
    self._ca.onUpdate("")
    self._ca.removeConfigurationSetNameListener(OpenRTM_aist.ConfigurationSetNameListenerType.ON_ACTIVATE_CONFIG_SET,
                                                listener)
    return

class ConfigParamListenerCallback(OpenRTM_aist.ConfigurationParamListener):
  def __init__(self):
    OpenRTM_aist.ConfigurationParamListener.__init__(self)
    return

  def __call__(self, config_set_name, config_param_name):
    print "ConfigParamListenerCallback called"
    return

class ConfigSetListenerCallback(OpenRTM_aist.ConfigurationSetListener):
  def __init__(self):
    OpenRTM_aist.ConfigurationSetListener.__init__(self)
    return

  def __call__(self, config_set):
    print "ConfigSetListenerCallback called"
    return

class ConfigSetNameListenerCallback(OpenRTM_aist.ConfigurationSetNameListener):
  def __init__(self):
    OpenRTM_aist.ConfigurationSetNameListener.__init__(self)
    return

  def __call__(self, config_set_name):
    print "ConfigSetNameListenerCallback called"
    return


############### test #################
if __name__ == '__main__':
        unittest.main()
