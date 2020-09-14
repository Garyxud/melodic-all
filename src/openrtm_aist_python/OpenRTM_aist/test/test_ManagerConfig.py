#!/usr/bin/env python
# -*- Python -*-

#
# \file test_ManagerConfig.py
# \brief test for RTC manager configuration
# \date $Date: $
# \author Shinji Kurihara
#
# Copyright (C) 2003
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

from ManagerConfig import *

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

class TestManagerConfig(unittest.TestCase) :
  
  def setUp(self):
    argv = (sys.argv[0], "-f", "./rtc.conf")
    self.mgrConf = ManagerConfig(argv)


  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return
  
  def test_configure(self):
    prop = OpenRTM_aist.Properties(defaults_str=configsample_spec)
    self.mgrConf.configure(prop)
    self.assertEqual(prop.getProperty("type_name"),"ConfigSample","Result failed.")

    
  def test_parseArgs(self):
    argv = (sys.argv[0], "-f", "./rtc.conf")
    self.mgrConf.parseArgs(argv)

    # Failed Pattern
    # argv = (sys.argv[0], "-c", "./rtc.conf")
    # self.mgrConf.parseArgs(3,argv)
    

  def test_findConfigFile(self):
    self.assertEqual(self.mgrConf.findConfigFile(),True,"Result failed.")


  def test_setSystemInformation(self):
    prop = OpenRTM_aist.Properties(defaults_str=configsample_spec)
    # Failed Pattern
    # self.assertEqual(prop.getProperty("manager.os.name"),"Linux","Result failed.")

    prop = self.mgrConf.setSystemInformation(prop)
    self.assertEqual(prop.getProperty("manager.os.name"),"Linux","Result failed.")
    #self.assertEqual(prop.getProperty("manager.os.name"),"Windows","Result failed.")


  def test_fileExist(self):
    self.assertEqual(self.mgrConf.fileExist("rtc.conf"),True,"Result failed.")
    self.assertEqual(self.mgrConf.fileExist("./rtc.conf"),True,"Result failed.")

    # Failed Pattern
    # self.assertEqual(self.mgrConf.fileExist("../rtc.conf"),True,"Result failed.")
    

############### test #################
if __name__ == '__main__':
        unittest.main()


