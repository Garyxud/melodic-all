#!/usr/bin/env python
# -*- Python -*- 

#  \file test_NVUtil.py
#  \brief test for NameValue and NVList utility functions
#  \date $Date: 2007/09/11$
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Noriaki Ando
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.

from omniORB import *
from omniORB import any
import sys
sys.path.insert(1,"../")

import unittest
import OpenRTM_aist
import SDOPackage, SDOPackage__POA

from NVUtil import *

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

class TestNVUtil(unittest.TestCase):
  def setUp(self):
    pass

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_newNV(self):
    self.assertEqual(newNV("long", 1).name, "long")
    self.assertEqual(newNV("long", 1).value.value(), 1)
    self.assertEqual(newNV("float", 1.2345).name, "float")
    self.assertEqual(newNV("float", 1.2345).value.value(), 1.2345)
    self.assertEqual(newNV("string", "test").name, "string")
    self.assertEqual(newNV("string", "test").value.value(), "test")


  def test_newNVBool(self):
    self.assertEqual(newNV("bool", True).name, "bool")
    self.assertEqual(newNV("bool", True).value.value(), True)
    

  def test_newNVOctet(self):
    self.assertEqual(newNV("oct", 0x01).name, "oct")
    self.assertEqual(newNV("oct", 0x01).value.value(), 0x01)


  def test_newNVAny(self):
    import omniORB.any
    self.assertEqual(omniORB.any.from_any(newNV("any",omniORB.any.to_any(12345)).value), 12345)


  def test_copyFromProperties(self):
    prop = OpenRTM_aist.Properties(defaults_str=configsample_spec)
    nv = []
    copyFromProperties(nv, prop)
    self.assertEqual(nv[0].name, "implementation_id")
    self.assertEqual(nv[0].value.value(), "ConfigSample")
    nv = [1,2,3]
    copyFromProperties(nv, prop)
    self.assertEqual(nv[0].name, "implementation_id")
    self.assertEqual(nv[0].value.value(), "ConfigSample")

    
  def test_copyToProperties(self):
    nv = [newNV("id0",0),newNV("id1",1),newNV("id2",2),newNV("id3",3)]
    prop = OpenRTM_aist.Properties(defaults_str=configsample_spec)
    copyToProperties(prop, nv)
    self.assertEqual(prop.getProperty("id0"), "0")
    self.assertEqual(prop.getProperty("id1"), "1")
    self.assertEqual(prop.getProperty("id2"), "2")
    self.assertEqual(prop.getProperty("id3"), "3")
    
    
  def test_toProperties(self):
    nv = [newNV("id0",0),newNV("id1",1),newNV("id2",2),newNV("id3",3)]
    prop = toProperties(nv)
    self.assertEqual(prop.getProperty("id0").value(), 0)
    self.assertEqual(prop.getProperty("id1").value(), 1)
    self.assertEqual(prop.getProperty("id2").value(), 2)
    self.assertEqual(prop.getProperty("id3").value(), 3)
    

  def test_find(self):
    nv = [newNV("id0",0),newNV("id1",1),newNV("id2",2),newNV("id3",3)]
    self.assertEqual(find(nv,"id0").value(), 0)
    self.assertEqual(find(nv,"id1").value(), 1)
    self.assertEqual(find(nv,"id2").value(), 2)
    self.assertEqual(find(nv,"id3").value(), 3)

    
  def test_find_index(self):
    nv = [newNV("id0",0),newNV("id1",1),newNV("id2",2),newNV("id3",3)]
    self.assertEqual(find_index(nv,"id0"),0)
    self.assertEqual(find_index(nv,"id1"),1)
    self.assertEqual(find_index(nv,"id2"),2)
    self.assertEqual(find_index(nv,"id3"),3)


  def test_isString(self):
    nv = [newNV("float",1.234),newNV("long",1234),newNV("string","test"),newNV("oct",0x01)]
    self.assertEqual(isString(nv,"float"),False)
    self.assertEqual(isString(nv,"long"),False)
    self.assertEqual(isString(nv,"string"),True)
    self.assertEqual(isString(nv,"oct"),False)
    

  def test_isStringValue(self):
    nv = [newNV("float",1.234),newNV("long",1234),newNV("string","test"),newNV("oct",0x01)]
    self.assertEqual(isStringValue(nv,"float",1.234),False)
    self.assertEqual(isStringValue(nv,"long",1234),False)
    self.assertEqual(isStringValue(nv,"string","test"),True)
    self.assertEqual(isStringValue(nv,"oct",0x01),False)


  def test_toString(self):
    nv = [newNV("float",1.234),newNV("long",1234),newNV("string","test"),newNV("oct",0x01)]
    self.assertEqual(toString(nv,"float"),"")
    self.assertEqual(toString(nv,"long"),"")
    self.assertEqual(toString(nv,"string"),"test")
    self.assertEqual(toString(nv,"oct"),"")
    

  def test_appendStringValue(self):
    nv = [newNV("string","test0, test1, test2")]
    self.assertEqual(appendStringValue(nv,"string","test2"),True)
    self.assertEqual(any.from_any(nv[0].value),"test0, test1, test2")
    self.assertEqual(appendStringValue(nv,"string","test3"),True)
    self.assertEqual(any.from_any(nv[0].value),"test0, test1, test2, test3")


  def test_append(self):
    list_ = [1,2,3]
    append(list_,[4,5,6])
    self.assertEqual(list_,[1,2,3,4,5,6])
    

  def test_dump(self):
    nv = [newNV("string","test0, test1, test2")]
    dump(nv)


############### test #################
if __name__ == '__main__':
        unittest.main()
