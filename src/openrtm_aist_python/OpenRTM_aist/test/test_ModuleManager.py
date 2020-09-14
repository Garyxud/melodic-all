#!/usr/bin/env python
# -*- Python -*-

#
# \file test_ModulesManager.py
# \brief Loadable modules manager class
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

import sys,os
sys.path.insert(1,"../")

import unittest

import OpenRTM_aist

from ModuleManager import *

configsample_spec = ["implementation_id", "ConfigSample",
                     "type_name",         "ConfigSample",
                     "description",       "Configuration example component",
                     "version",           "1.0",
                     "vendor",            "Shinji Kurihara, AIST",
                     "category",          "example",
                     "activity_type",     "DataFlowComponent",
                     "max_instance",      "10",
                     "language",          "Python",
                     "lang_type",         "script",
                     # Configuration variables
                     "conf.default.int_param0", "0",
                     "conf.default.int_param1", "1",
                     "conf.default.double_param0", "0.11",
                     "conf.default.double_param1", "9.9",
                     "conf.default.str_param0", "hoge",
                     "conf.default.str_param1", "dara",
                     "conf.default.vector_param0", "0.0,1.0,2.0,3.0,4.0",
                     "manager.modules.abs_path_allowed",   "YES",
                     ""]


class TestModuleManager(unittest.TestCase):
  def setUp(self):
    self.mm = ModuleManager(OpenRTM_aist.Properties(defaults_str=configsample_spec))

  def tearDown(self):
    del self.mm
    OpenRTM_aist.Manager.instance().shutdownManager()
    
    
  def test_load_unload(self):
    try:
      path = os.getcwd()
      self.mm.load(path+"/hoge.py","echo")
      self.mm.unload(path+"/hoge.py")

      self.mm.load("hoge","echo")
      self.mm.unload("hoge")

      self.mm.load("hoge.py","echo")
      self.mm.unload("hoge.py")

      self.mm.load("./hoge.py","echo")
      self.mm.unload("./hoge.py")

      # Failure Pattern
      #self.mm.load("sample")
    except:
      print "exception."
    return
    

  def test_unloadAll(self):
    self.mm.unloadAll()
    return
    

  def test_symbol(self):
    path = os.getcwd()
    self.mm.load(path+"/hoge.py","echo")
    self.mm.symbol(path+"/hoge.py","echo")()
    self.mm.unload(path+"/hoge.py")

    self.mm.load("hoge","echo")
    self.mm.symbol("hoge","echo")()
    self.mm.unload("hoge")

    self.mm.load("hoge.py","echo")
    self.mm.symbol("hoge.py","echo")()
    self.mm.unload("hoge.py")

    
  def test_setLoadpath(self):
    self.mm.setLoadpath(["/usr/lib/python/site-packages","."])
    return
  
  def test_getLoadPath(self):
    self.mm.setLoadpath(["/usr/lib/python/site-packages","."])
    self.assertEqual(self.mm.getLoadPath()[0],"/usr/lib/python/site-packages")
    return

    
  def test_addLoadpath(self):
    self.mm.setLoadpath(["/usr/lib/python/site-packages","."])
    self.mm.addLoadpath(["/usr/local/lib/python/site-packages"])
    self.assertEqual(self.mm.getLoadPath()[0],"/usr/lib/python/site-packages")
    self.assertEqual(self.mm.getLoadPath()[-1],"/usr/local/lib/python/site-packages")
    return

  
  def test_getLoadedModules(self):
    self.mm.load("hoge","echo")
    self.assertNotEqual(self.mm.getLoadedModules()[0],None)
    return
    

  def test_allowAbsolutePath(self):
    self.mm.allowAbsolutePath()
    return

    
  def test_disallowAbsolutePath(self):
    self.mm.disallowAbsolutePath()
    return

    
  def test_allowModuleDownload(self):
    self.mm.allowModuleDownload()
    return

    
  def test_disallowModuleDownload(self):
    self.mm.disallowModuleDownload()
    return

    
  def test_findFile(self):
    self.assertEqual(self.mm.findFile("hoge",["."]),"hoge")
    self.assertEqual(self.mm.findFile("hoge.py",["."]),"hoge.py")
    self.assertEqual(self.mm.findFile("hogehoge",["."]),"")
    return

    
  def test_fileExist(self):
    self.assertEqual(self.mm.fileExist("hoge.py"),True)
    self.assertEqual(self.mm.fileExist("./hoge.py"),True)
    self.assertEqual(self.mm.fileExist("hoge"),True)
    self.assertEqual(self.mm.fileExist("./hoge"),True)
    self.assertEqual(self.mm.fileExist("hogehoge"),False)
    return

    
  def test_getInitFuncName(self):
    self.mm.getInitFuncName("hoge.py")
    return
    

  def test_getRtcProfile(self):
    self.assertEqual(self.mm._ModuleManager__getRtcProfile("./ConfigSample.py"),None)
    self.assertEqual(self.mm._ModuleManager__getRtcProfile("ConfigSample.py"),None)
    self.assertEqual(self.mm._ModuleManager__getRtcProfile("ConfigSample"),None)
    return


  def test_getLoadableModules(self):
    self.mm.setLoadpath([".","./","../"])
    self.assertNotEqual(self.mm.getLoadableModules(),[])
    return



############### test #################
if __name__ == '__main__':
        unittest.main()
