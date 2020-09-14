#!/usr/bin/env python
# -*- Python -*-
#
# \file test_ECFactory.py
# \brief test for ExecutionContext Factory class
# \date $Date: $
# \author Shinji Kurihara
# 
# Copyright (C) 2007
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
# 


import sys
sys.path.insert(1,"../")

import unittest

from ECFactory import *
import OpenRTM_aist


class TestECFactoryPython(unittest.TestCase):

  def setUp(self):
    self.ecfact = ECFactoryPython("test", self.create_func, self.del_func)
    return
  
  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    del self
    return

  def create_func(self):
    print "create_func"

  def del_func(self, ec):
    print "del_func"
    
  def test_name(self):
    name = self.ecfact.name()
    self.assertEqual(name,"test", "name is false.")

  def test_create(self):
    self.ecfact.create()


  def test_destroy(self):
    self.ecfact.destroy("hoge")
    
############### test #################
if __name__ == '__main__':
        unittest.main()

