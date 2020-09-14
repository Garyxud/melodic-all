#!/usr/bin/env python
# -*- Python -*-

#
# \file test_NumberingPolicy.py
# \brief Object numbering policy class
# \date $Date: 2007/08/23$
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

from NumberingPolicy import *
import OpenRTM_aist


class TestDefaultNumberingPolicy(unittest.TestCase):
  def setUp(self):
    self.__dnp = DefaultNumberingPolicy()

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_onCreate(self):
    self.assertEqual(self.__dnp.onCreate("test0"),"0")
    self.assertEqual(self.__dnp.onCreate("test1"),"1")
    self.assertEqual(self.__dnp.onCreate("test1"),"2")


  
  def test_onDelete(self):
    self.__dnp.onCreate("test")
    self.__dnp.onCreate("test0")
    self.__dnp.onDelete("test")
    self.assertEqual(self.__dnp.onCreate("test1"),"0")
    self.assertEqual(self.__dnp.onCreate("test"),"2")
    


  def test_find(self):
    pass
  

############### test #################
if __name__ == '__main__':
        unittest.main()
