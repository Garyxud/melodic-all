#!/usr/bin/env python
# -*- Python -*-

#
# @file test_ObjectManager.py
# @brief test for Object management class
# @date $Date: $
# @author Shinji Kurihara
#
# Copyright (C) 2003-2007
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id: $
#

import sys
sys.path.insert(1,"../")

import unittest

from ObjectManager import *
import OpenRTM_aist

class test_obj:
  def __init__(self, name):
    self.name = name

  def getInstanceName(self):
    return self.name

  def getProperty(self,name):
    return name
  

class TestObjectManager(unittest.TestCase):

  class InstanceName :
    def __init__(self, name=None, factory=None):
      if factory != None:
        self._name = factory.getInstanceName()
      elif name != None:
        self._name = name
      
    #def func(self, factory):
    def __call__(self, factory):
      return self._name == factory.getInstanceName()


  class ModuleFactories :
    def __init__(self):
      self._modlist = []

    
    #def func(self, f):
    def __call__(self, f):
      self._modlist.append(f.getProperty("implementation_id"))

      
  def setUp(self):
    self.obj = ObjectManager(self.InstanceName)

    
  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_registerObject(self):
    obj = test_obj("test")
    self.assertEqual(self.obj.registerObject(obj),True,"Result failed.")
    self.assertEqual(self.obj.registerObject(obj),False,"Result failed.")


  def test_unregisterObject(self):
    self.test_registerObject()
    self.assertEqual(self.obj.unregisterObject("test").getInstanceName(),"test","Result failed.")
    self.assertEqual(self.obj.unregisterObject("test"),None,"Result failed.")
    # Failed Pattern
    # self.assertEqual(self.obj.unregisterObject("test").getInstanceName(),"test","Result failed.")


  def test_find(self):
    self.test_registerObject()
    self.assertEqual(self.obj.find("test").getInstanceName(),"test","Result failed.")
    self.assertEqual(self.obj.find("testtest"),None,"Result failed.")


  def test_getObjects(self):
    obj = test_obj("test0")
    self.obj.registerObject(obj)
    obj = test_obj("test1")
    self.obj.registerObject(obj)
    obj = test_obj("test2")
    self.obj.registerObject(obj)
    self.assertEqual(len(self.obj.getObjects()),3,"Result failed.")
    self.assertEqual(self.obj.getObjects()[0].getInstanceName(),"test0","Result failed.")
    self.assertEqual(self.obj.getObjects()[1].getInstanceName(),"test1","Result failed.")
    self.assertEqual(self.obj.getObjects()[2].getInstanceName(),"test2","Result failed.")
    

  def test_for_each(self):
    obj = test_obj("test0")
    self.obj.registerObject(obj)
    obj = test_obj("test1")
    self.obj.registerObject(obj)
    obj = test_obj("test2")
    self.obj.registerObject(obj)

    self.assertEqual(len(self.obj.for_each(self.ModuleFactories)._modlist),3,"Result failed.")


############### test #################
if __name__ == '__main__':
  unittest.main()
