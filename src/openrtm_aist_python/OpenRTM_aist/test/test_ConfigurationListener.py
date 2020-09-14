#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file test ConfigurationListener.py
# @brief test for ConfigurationListener class
# @date $Date$
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

from ConfigurationListener import *
import OpenRTM_aist

config_set = ["conf.default.int_param0", "0",
              "conf.default.int_param1", "1",
              "conf.default.double_param0", "0.11",
              "conf.default.double_param1", "9.9",
              "conf.default.str_param0", "hoge",
              "conf.default.str_param1", "dara",
              "conf.default.vector_param0", "0.0,1.0,2.0,3.0,4.0",
              ""]

class MockConfigurationParamListener(ConfigurationParamListener):
  def __init__(self):
    ConfigurationParamListener.__init__(self)
    return

  def __call__(self, config_set_name, config_param_name):
    return (config_set_name, config_param_name)

class MockConfigurationSetListener(ConfigurationSetListener):
  def __init__(self):
    ConfigurationSetListener.__init__(self)
    return

  def __call__(self, config_set):
    return config_set

class MockConfigurationSetNameListener(ConfigurationSetNameListener):
  def __init__(self):
    ConfigurationSetNameListener.__init__(self)
    return

  def __call__(self, config_set_name):
    return config_set_name


class TestListener(unittest.TestCase):
  def setUp(self):
    return

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_ConfigurationParamListener_toString(self):
    self.assertEqual("ON_UPDATE_CONFIG_PARAM",
                     ConfigurationParamListener.toString(
        ConfigurationParamListenerType.ON_UPDATE_CONFIG_PARAM))
    return

  def test_ConfigurationSetListener_toString(self):
    self.assertEqual("ON_SET_CONFIG_SET",
                     ConfigurationSetListener.toString(
        ConfigurationSetListenerType.ON_SET_CONFIG_SET))
    self.assertEqual("ON_ADD_CONFIG_SET",
                     ConfigurationSetListener.toString(
        ConfigurationSetListenerType.ON_ADD_CONFIG_SET))
    return

  def test_ConfigurationSetNameListener_toString(self):
    self.assertEqual("ON_UPDATE_CONFIG_SET",
                     ConfigurationSetNameListener.toString(
        ConfigurationSetNameListenerType.ON_UPDATE_CONFIG_SET))
    self.assertEqual("ON_REMOVE_CONFIG_SET",
                     ConfigurationSetNameListener.toString(
        ConfigurationSetNameListenerType.ON_REMOVE_CONFIG_SET))
    self.assertEqual("ON_ACTIVATE_CONFIG_SET",
                     ConfigurationSetNameListener.toString(
        ConfigurationSetNameListenerType.ON_ACTIVATE_CONFIG_SET))
    return

  def test_ConfigurationParamListenerHolder(self):
    configparams = ConfigurationListeners()
    listener = MockConfigurationParamListener()
    configparams.configparam_[0].addListener(listener,True)
    configparams.configparam_[0].notify("config_set_name","config_param_name")
    configparams.configparam_[0].removeListener(listener)
    return

  def test_ConfigurationSetListenerHolder(self):
    configsetss = ConfigurationListeners()
    listener = MockConfigurationSetListener()
    configsets.configset_[0].addListener(listener,True)
    prop = OpenRTM_aist.Properties(defaults_str=config_set)
    configsets.configset_[0].notify(prop)
    configsets.configset_[0].removeListener(listener)
    return
  
  def test_ConfigurationSetNameListenerHolder(self):
    configsetnames = ConfigurationListeners()
    listener = MockConfigurationSetNameListener()
    configsetnames.configsetname_[0].addListener(listener,True)
    configsetnames.configsetname_[0].notify("config_set_name")
    configsetnames.configsetname_[0].removeListener(listener)
    return


############### test #################
if __name__ == '__main__':
  unittest.main()

