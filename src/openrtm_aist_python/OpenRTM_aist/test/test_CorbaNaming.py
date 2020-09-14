#!/usr/bin/env python
# -*- Python -*-


## \file test_CorbaNaming.py
## \brief test for CORBA naming service helper class
## \author Shinji Kurihara
#
# Copyright (C) 2006
#     Noriaki Ando
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

from CorbaNaming import *
import OpenRTM_aist

class TestCorbaNaming(unittest.TestCase):

  def setUp(self):
    self.orb = OpenRTM_aist.Manager.instance().getORB()
    self.naming = CorbaNaming(self.orb, "localhost:2809")
    #self.naming.clearAll()
    self.name_list = []
    self.name_list.append(CosNaming.NameComponent("Mon","mon_cxt"))
    self.name_list.append(CosNaming.NameComponent("Tus","tus_cxt"))
    self.name_list.append(CosNaming.NameComponent("Wed","wed_cxt"))
    self.name_list.append(CosNaming.NameComponent("Thu","thu_cxt"))
    self.name_list.append(CosNaming.NameComponent("Fri","fri_cxt"))
    self.name_list.append(CosNaming.NameComponent("Sat","sat_cxt"))
    self.name_list.append(CosNaming.NameComponent("Sun","sun_cxt"))
      
    return
  

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    del self
    return

  def test_init(self):
    try:
      self.naming.init("localhost")
    except:
      print "Exeption at naming init."


  def test_bind(self):
    self.naming.clearAll()
    self.bind_test_sequence(self.name_list)
    

  def bind_test_sequence(self, sname):
    obj = self.naming.newContext()
    self.naming.bind(sname, obj)

    # resolve test
    new_obj = self.naming.resolve(sname)
    self.assertEqual(obj._is_equivalent(new_obj), True,"Resolve failed.")
    return True


  def test_bindByString(self):
    name = "localhost.host_cxt\/Manager123456.mgr_cxt/MobileRobot.cat_cxt/Kani0.rtc"

    lname = []
    lname.append(CosNaming.NameComponent("localhost.host_cxt\/Manager123456","mgr_cxt"))
    lname.append(CosNaming.NameComponent("MobileRobot","cat_cxt"))
    lname.append(CosNaming.NameComponent("Kani0","rtc"))

    obj = self.naming.newContext()
    self.naming.bindByString(name, obj)
    new_obj = self.naming.resolve(name)
    self.assertEqual(obj._is_equivalent(new_obj), True,"Resolve failed.")
    new_obj = self.naming.resolve(lname)
    self.assertEqual(obj._is_equivalent(new_obj), True,"Resolve failed.")


  #def test_bindRecursive(self):
  # pass

  def test_rebind(self):
    obj = self.naming.newContext()

    self.naming.bind(self.name_list, obj)
    new_obj = self.naming.resolve(self.name_list)
    self.assertEqual(obj._is_equivalent(new_obj), True,"Resolve failed.")

    self.naming.rebind(self.name_list, obj)
    new_obj = self.naming.resolve(self.name_list)
    self.assertEqual(obj._is_equivalent(new_obj), True,"Resolve failed.")
    

  #def test_rebindByString(self):
  # pass
    

  def test_rebindRecursive(self):
    obj = self.naming.newContext()

    self.naming.bind(self.name_list, obj)
    new_obj = self.naming.resolve(self.name_list)
    self.assertEqual(obj._is_equivalent(new_obj), True,"Resolve failed.")

    self.naming.rebindRecursive(self.naming.getRootContext(), self.name_list, obj)
    new_obj = self.naming.resolve(self.name_list)
    self.assertEqual(obj._is_equivalent(new_obj), True,"Resolve failed.")


  def test_bindContext(self):
    obj = self.naming.newContext()
    name = []
    name.append(CosNaming.NameComponent("bindcxt","bindcxt_cxt"))

    self.naming.bindContext("bindcxt.bindcxt_cxt", obj)
    new_obj = self.naming.resolve(name)
    self.assertEqual(obj._is_equivalent(new_obj), True,"Resolve failed.")


  def test_unbind(self):
    obj = self.naming.newContext()

    self.naming.bind(self.name_list, obj)
    self.naming.unbind(self.name_list)
  

  def test_bindNewContext(self):
    name = []
    name.append(CosNaming.NameComponent("newContext","new_cxt"))

    self.naming.bindNewContext(name)
    new_obj = self.naming.resolve(name)


  def test_destroy(self):
    obj = self.naming.newContext()
    name = []
    name.append(CosNaming.NameComponent("destroy","destroy_cxt"))

    self.naming.bind(name, obj)
    self.naming.destroy(obj)


  def test_destroyRecursive(self):
    obj = self.naming.newContext()
    name = []
    name.append(CosNaming.NameComponent("desRec0","desRec0_cxt"))
    name.append(CosNaming.NameComponent("desRec1","desRec1_cxt"))
    name.append(CosNaming.NameComponent("desRec2","desRec2_cxt"))

    self.naming.bind(name, obj)
    self.naming.destroyRecursive(obj)

  def test_list(self):
    bl = []
    bi = []
    self.naming.list(self.naming.getRootContext(), 100, bl, bi)
    print bl[0][0].binding_name[0].id
  

  def test_toString(self):
    name = []
    name.append(CosNaming.NameComponent("desRec0","desRec0_cxt"))
    name.append(CosNaming.NameComponent("desRec1","desRec1_cxt"))
    name.append(CosNaming.NameComponent("desRec2","desRec2_cxt"))

    string_name = self.naming.toString(name)
    self.assertEqual(str("desRec0.desRec0_cxt/desRec1.desRec1_cxt/desRec2.desRec2_cxt/"),str(string_name))
    print ".",string_name,"."

############### test #################
if __name__ == '__main__':
  unittest.main()
