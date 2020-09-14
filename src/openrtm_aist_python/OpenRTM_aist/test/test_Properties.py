#!/usr/bin/env python
# -*- Python -*-

#
# @file test_Properties.py
# @brief test for Property list class (derived from Java Properties)
# @date $Date: $
# @author Shinji Kurihara
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

from Properties import *
import OpenRTM_aist


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

class TestProperties(unittest.TestCase) :

  def setUp(self):

    self.prop = Properties(defaults_str=configsample_spec)

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return
  
  def test_getName(self):
    self.assertEqual(self.prop.getName(), "", "Result failed.")


  def test_getVlue(self):
    self.assertEqual(self.prop.getValue(), "", "Result failed.")
  

  def test_getDefaultValue(self):
    self.assertEqual(self.prop.getDefaultValue(), "", "Result failed.")



  def test_getLeaf(self):
    #print self.prop.getLeaf()[0].getName()
    #print self.prop.getLeaf()[0].getValue()

    self.assertEqual(self.prop.getLeaf()[0].getName(), "implementation_id", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[1].getName(), "type_name", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[2].getName(), "description", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[3].getName(), "version", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[4].getName(), "vendor", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[5].getName(), "category", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[6].getName(), "activity_type", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[7].getName(), "max_instance", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[8].getName(), "language", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[9].getName(), "lang_type", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[10].getLeaf()[0].getLeaf()[0].getName(), "int_param0", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[10].getLeaf()[0].getLeaf()[1].getName(), "int_param1", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[10].getLeaf()[0].getLeaf()[2].getName(), "double_param0", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[10].getLeaf()[0].getLeaf()[3].getName(), "double_param1", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[10].getLeaf()[0].getLeaf()[4].getName(), "str_param0", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[10].getLeaf()[0].getLeaf()[5].getName(), "str_param1", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[10].getLeaf()[0].getLeaf()[6].getName(), "vector_param0", "Result failed.")

    # Failed Patern
    # self.assertEqual(self.prop.getLeaf()[10].getName(), "conf.default.int_param0", "Result failed.")


  def test_getRoot(self):
    self.assertEqual(self.prop.getRoot(), None, "Result failed.")
    self.assertEqual(self.prop.getLeaf()[10].getRoot().getRoot(), None, "Result failed.")
    self.assertEqual(self.prop.getLeaf()[10].getLeaf()[0].getRoot().getName(), "conf", "Result failed.")
    self.assertEqual(self.prop.getLeaf()[10].getLeaf()[0].getLeaf()[0].getRoot().getName(), "default", "Result failed.")
  

  def test_getProperty(self):
    self.assertEqual(self.prop.getProperty("implementation_id"), "ConfigSample", "Result failed.")
    self.assertEqual(self.prop.getProperty("type_name"),  "ConfigSample", "Result failed.")
    self.assertEqual(self.prop.getProperty("description"), "Configuration example component","Result failed.")
    self.assertEqual(self.prop.getProperty("version"), "1.0","Result failed.")
    self.assertEqual(self.prop.getProperty("vendor"),"Shinji Kurihara, AIST", "Result failed.")
    self.assertEqual(self.prop.getProperty("category"), "example","Result failed.")
    self.assertEqual(self.prop.getProperty("activity_type"), "DataFlowComponent", "Result failed.")
    self.assertEqual(self.prop.getProperty("max_instance"), "10", "Result failed.")
    self.assertEqual(self.prop.getProperty("language"), "C++", "Result failed.")
    self.assertEqual(self.prop.getProperty("lang_type"), "compile", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.int_param0"), "0", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.int_param1"), "1", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.double_param0"), "0.11", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.double_param1"), "9.9", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.str_param0"), "hoge", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.str_param1"), "dara", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.vector_param0"), "0.0,1.0,2.0,3.0,4.0","Result failed.")

    # Failed Pattern
    # self.assertEqual(self.prop.getProperty("int_param0"), "0", "Result failed.")
    # self.assertEqual(self.prop.getProperty("int_param1"), "1", "Result failed.")
    # self.assertEqual(self.prop.getProperty("double_param0"), "0.11", "Result failed.")
    # self.assertEqual(self.prop.getProperty("double_param1"), "9.9", "Result failed.")
    # self.assertEqual(self.prop.getProperty("str_param0"), "hoge", "Result failed.")
    # self.assertEqual(self.prop.getProperty("str_param1"), "dara", "Result failed.")
    # self.assertEqual(self.prop.getProperty("vector_param0"), "0.0,1.0,2.0,3.0,4.0","Result failed.")

  
  def test_getDefault(self):
    self.assertEqual(self.prop.getDefault("implementation_id"), "ConfigSample", "Result failed.")
    self.assertEqual(self.prop.getDefault("type_name"),  "ConfigSample", "Result failed.")
    self.assertEqual(self.prop.getDefault("description"), "Configuration example component","Result failed.")
    self.assertEqual(self.prop.getDefault("version"), "1.0","Result failed.")
    self.assertEqual(self.prop.getDefault("vendor"),"Shinji Kurihara, AIST", "Result failed.")
    self.assertEqual(self.prop.getDefault("category"), "example","Result failed.")
    self.assertEqual(self.prop.getDefault("activity_type"), "DataFlowComponent", "Result failed.")
    self.assertEqual(self.prop.getDefault("max_instance"), "10", "Result failed.")
    self.assertEqual(self.prop.getDefault("language"), "C++", "Result failed.")
    self.assertEqual(self.prop.getDefault("lang_type"), "compile", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.int_param0"), "0", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.int_param1"), "1", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.double_param0"), "0.11", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.double_param1"), "9.9", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.str_param0"), "hoge", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.str_param1"), "dara", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.vector_param0"), "0.0,1.0,2.0,3.0,4.0","Result failed.")
    
  
  def test_setProperty(self):
    self.prop.setProperty("implementation_id", "ConfigSample_test")
    self.prop.setProperty("type_name",  "ConfigSample_test")
    self.prop.setProperty("description", "Configuration example component test")
    self.prop.setProperty("version", "2.0")
    self.prop.setProperty("vendor","Shinji Kurihara")
    self.prop.setProperty("category", "example_test")
    self.prop.setProperty("activity_type", "DataFlowComponent")
    self.prop.setProperty("max_instance", "1")
    self.prop.setProperty("language", "C++")
    self.prop.setProperty("lang_type", "compile")
    self.prop.setProperty("conf.default.int_param0", "111")
    self.prop.setProperty("conf.default.int_param1", "222")
    self.prop.setProperty("conf.default.double_param0", "2.22")
    self.prop.setProperty("conf.default.double_param1", "9.99")
    self.prop.setProperty("conf.default.str_param0", "hogehoge")
    self.prop.setProperty("conf.default.str_param1", "daradara")
    self.prop.setProperty("conf.default.vector_param0", "0.0,1.1,2.2,3.3,4.4")

    self.assertEqual(self.prop.getProperty("implementation_id"), "ConfigSample_test", "Result failed.")
    self.assertEqual(self.prop.getProperty("type_name"),  "ConfigSample_test", "Result failed.")
    self.assertEqual(self.prop.getProperty("description"), "Configuration example component test","Result failed.")
    self.assertEqual(self.prop.getProperty("version"), "2.0","Result failed.")
    self.assertEqual(self.prop.getProperty("vendor"),"Shinji Kurihara", "Result failed.")
    self.assertEqual(self.prop.getProperty("category"), "example_test","Result failed.")
    self.assertEqual(self.prop.getProperty("activity_type"), "DataFlowComponent", "Result failed.")
    self.assertEqual(self.prop.getProperty("max_instance"), "1", "Result failed.")
    self.assertEqual(self.prop.getProperty("language"), "C++", "Result failed.")
    self.assertEqual(self.prop.getProperty("lang_type"), "compile", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.int_param0"), "111", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.int_param1"), "222", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.double_param0"), "2.22", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.double_param1"), "9.99", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.str_param0"), "hogehoge", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.str_param1"), "daradara", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.default.vector_param0"), "0.0,1.1,2.2,3.3,4.4","Result failed.")
  

  def test_setDefault(self):
    self.prop.setDefault("implementation_id", "ConfigSample")
    self.prop.setDefault("type_name",  "ConfigSample")
    self.prop.setDefault("description", "Configuration example component")
    self.prop.setDefault("version", "1.0")
    self.prop.setDefault("vendor","Shinji Kurihara")
    self.prop.setDefault("category", "example")
    self.prop.setDefault("activity_type", "DataFlowComponent")
    self.prop.setDefault("max_instance", "10")
    self.prop.setDefault("language", "C++")
    self.prop.setDefault("lang_type", "compile")
    self.prop.setDefault("conf.default.int_param0", "0")
    self.prop.setDefault("conf.default.int_param1", "1")
    self.prop.setDefault("conf.default.double_param0", "0.11")
    self.prop.setDefault("conf.default.double_param1", "9.9")
    self.prop.setDefault("conf.default.str_param0", "hoge")
    self.prop.setDefault("conf.default.str_param1", "dara")
    self.prop.setDefault("conf.default.vector_param0", "0.0,1.0,2.0,3.0,4.0")

    self.assertEqual(self.prop.getDefault("implementation_id"), "ConfigSample", "Result failed.")
    self.assertEqual(self.prop.getDefault("type_name"),  "ConfigSample", "Result failed.")
    self.assertEqual(self.prop.getDefault("description"), "Configuration example component","Result failed.")
    self.assertEqual(self.prop.getDefault("version"), "1.0","Result failed.")
    self.assertEqual(self.prop.getDefault("vendor"),"Shinji Kurihara", "Result failed.")
    self.assertEqual(self.prop.getDefault("category"), "example","Result failed.")
    self.assertEqual(self.prop.getDefault("activity_type"), "DataFlowComponent", "Result failed.")
    self.assertEqual(self.prop.getDefault("max_instance"), "10", "Result failed.")
    self.assertEqual(self.prop.getDefault("language"), "C++", "Result failed.")
    self.assertEqual(self.prop.getDefault("lang_type"), "compile", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.int_param0"), "0", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.int_param1"), "1", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.double_param0"), "0.11", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.double_param1"), "9.9", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.str_param0"), "hoge", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.str_param1"), "dara", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.vector_param0"), "0.0,1.0,2.0,3.0,4.0","Result failed.")

  
  def test_setDefaults(self):
    self.prop.setDefaults(configsample_spec)
  
    self.assertEqual(self.prop.getDefault("implementation_id"), "ConfigSample", "Result failed.")
    self.assertEqual(self.prop.getDefault("type_name"),  "ConfigSample", "Result failed.")
    self.assertEqual(self.prop.getDefault("description"), "Configuration example component","Result failed.")
    self.assertEqual(self.prop.getDefault("version"), "1.0","Result failed.")
    self.assertEqual(self.prop.getDefault("vendor"),"Shinji Kurihara, AIST", "Result failed.")
    self.assertEqual(self.prop.getDefault("category"), "example","Result failed.")
    self.assertEqual(self.prop.getDefault("activity_type"), "DataFlowComponent", "Result failed.")
    self.assertEqual(self.prop.getDefault("max_instance"), "10", "Result failed.")
    self.assertEqual(self.prop.getDefault("language"), "C++", "Result failed.")
    self.assertEqual(self.prop.getDefault("lang_type"), "compile", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.int_param0"), "0", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.int_param1"), "1", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.double_param0"), "0.11", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.double_param1"), "9.9", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.str_param0"), "hoge", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.str_param1"), "dara", "Result failed.")
    self.assertEqual(self.prop.getDefault("conf.default.vector_param0"), "0.0,1.0,2.0,3.0,4.0","Result failed.")


  def test_list(self):
    #self.prop.list(sys.stdout)
    pass
  

  def test_load(self):
    fp = file("configsample.conf","r")
    self.prop.load(fp)
    self.assertEqual(self.prop.getProperty("conf.mode0.int_param0"), "12345", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.mode0.int_param1"), "98765", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.mode0.double_param0"), "3.141592653589793238462643383279")
    self.assertEqual(self.prop.getProperty("conf.mode0.double_param1"), "2.718281828459045235360287471352", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.mode0.str_param0"), "bar", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.mode0.str_param1"), "foo", "Result failed.")
    self.assertEqual(self.prop.getProperty("conf.mode0.vector_param0"), "0.0,0.1,0.2,0.3,0.4","Result failed.")


  def test_save(self):
    fp = file("test_save.data","w")
    self.prop.save(fp,"test Properties")


  def test_store(self):
    fp = file("test_store.data","w")
    self.prop.store(fp,"test Properties")

  
  def test_propertyNames(self):
    keys = self.prop.propertyNames()
    self.assertEqual(keys[0],"implementation_id", "Result failed.")
    self.assertEqual(keys[1],"type_name", "Result failed.")
    self.assertEqual(keys[2],"description","Result failed.")
    self.assertEqual(keys[3],"version","Result failed.")
    self.assertEqual(keys[4],"vendor", "Result failed.")
    self.assertEqual(keys[5],"category","Result failed.")
    self.assertEqual(keys[6],"activity_type", "Result failed.")
    self.assertEqual(keys[7],"max_instance", "Result failed.")
    self.assertEqual(keys[8],"language", "Result failed.")
    self.assertEqual(keys[9],"lang_type", "Result failed.")
    self.assertEqual(keys[10],"conf.default.int_param0", "Result failed.")
    self.assertEqual(keys[11],"conf.default.int_param1", "Result failed.")
    self.assertEqual(keys[12],"conf.default.double_param0", "Result failed.")
    self.assertEqual(keys[13],"conf.default.double_param1", "Result failed.")
    self.assertEqual(keys[14],"conf.default.str_param0", "Result failed.")
    self.assertEqual(keys[15],"conf.default.str_param1", "Result failed.")
    self.assertEqual(keys[16],"conf.default.vector_param0","Result failed.")
    

  def test_size(self):
    self.assertEqual(self.prop.size(),17,"Result failed.")
  
  
  def test_findNode(self):
    self.assertEqual(self.prop.findNode("implementation_id").getDefaultValue(), "ConfigSample", "Result failed.")
  
  def test_getNode(self):
    self.assertEqual(self.prop.getNode("implementation_id").getDefaultValue(), "ConfigSample", "Result failed.")
    self.assertEqual(self.prop.getNode("type_name").getDefaultValue(),  "ConfigSample", "Result failed.")
    self.assertEqual(self.prop.getNode("description").getDefaultValue(), "Configuration example component","Result failed.")
    self.assertEqual(self.prop.getNode("version").getDefaultValue(), "1.0","Result failed.")
    self.assertEqual(self.prop.getNode("vendor").getDefaultValue(),"Shinji Kurihara, AIST", "Result failed.")
    self.assertEqual(self.prop.getNode("category").getDefaultValue(), "example","Result failed.")
    self.assertEqual(self.prop.getNode("activity_type").getDefaultValue(), "DataFlowComponent", "Result failed.")
    self.assertEqual(self.prop.getNode("max_instance").getDefaultValue(), "10", "Result failed.")
    self.assertEqual(self.prop.getNode("language").getDefaultValue(), "C++", "Result failed.")
    self.assertEqual(self.prop.getNode("lang_type").getDefaultValue(), "compile", "Result failed.")
    self.assertEqual(self.prop.getNode("conf.default.int_param0").getDefaultValue(), "0", "Result failed.")
    self.assertEqual(self.prop.getNode("conf.default.int_param1").getDefaultValue(), "1", "Result failed.")
    self.assertEqual(self.prop.getNode("conf.default.double_param0").getDefaultValue(), "0.11", "Result failed.")
    self.assertEqual(self.prop.getNode("conf.default.double_param1").getDefaultValue(), "9.9", "Result failed.")
    self.assertEqual(self.prop.getNode("conf.default.str_param0").getDefaultValue(), "hoge", "Result failed.")
    self.assertEqual(self.prop.getNode("conf.default.str_param1").getDefaultValue(), "dara", "Result failed.")
    self.assertEqual(self.prop.getNode("conf.default.vector_param0").getDefaultValue(), "0.0,1.0,2.0,3.0,4.0","Result failed.")


  def test_createNode(self):
    self.assertEqual(self.prop.createNode("conf.default.int_param.0"),True, "Result failed.")
    self.assertEqual(self.prop.createNode("conf.add.int_param1"),True, "Result failed.")
    self.assertEqual(len(self.prop.getNode("conf.default").leaf), 8, "Result failed.")
    self.assertEqual(len(self.prop.getNode("conf.add").leaf), 1, "Result failed.")

  
  def test_removeNode(self):
    node = self.prop.getNode("conf.default")
    node.removeNode("int_param0")
    self.assertEqual( len(self.prop.getNode("conf.default").leaf),6, "Result failed.")
  
    node = self.prop.getNode("conf")
    node.removeNode("default")
    self.assertEqual( len(self.prop.getNode("conf").leaf),0, "Result failed.")


  def test_hasKey(self):
    node = self.prop.getNode("conf")
    self.assertEqual(len(node.hasKey("default").leaf),7, "Result failed.")

    self.assertEqual(self.prop.hasKey("default"),None, "Result failed.")


  def test_clear(self):
    self.prop.clear()
    self.assertEqual(self.prop.getProperty("implementation_id"), "", "Result failed.")

    # Failed Pattern
    # self.assertEqual(self.prop.getProperty("implementation_id"), "ConfigSample", "Result failed.")


  def test_splitKeyValue(self):
    key=[]
    val=[]
    self.prop.splitKeyValue("key:value",key,val)
    self.assertEqual(key[0],"key")
    self.assertEqual(val[0],"value")


  def test_split(self):
    val=[]
    self.prop.split("test.split,hoge",".",val)
    
    self.assertEqual(val, ["test","split,hoge"])


  def test_mergeProperties(self):
    self.prop.mergeProperties(self.prop)
    self.assertEqual(self.prop.getProperty("implementation_id"), "ConfigSample", "Result failed.")

  
  def test_dump(self):
    print self.prop
  


############### test #################
if __name__ == '__main__':
        unittest.main()
