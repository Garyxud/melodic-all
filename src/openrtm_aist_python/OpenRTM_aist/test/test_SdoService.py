#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_SdoService.py
#  \brief test for SDO Service administration class
#  \date $Date: 2007/09/12 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 


import sys
sys.path.insert(1,"../")
sys.path.insert(1,"../RTM_IDL")

import unittest

from SdoService import *
import OpenRTM_aist


class TestSDOServiceProfile(unittest.TestCase):
  def setUp(self):
    self.sdosp = SDOServiceProfile()

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_getProfile(self):
    sdosp = SDOServiceProfile("test", "test_type")
    prof = sdosp.getProfile()
    self.assertEqual(prof.id,"test")
    self.assertEqual(prof.type,"test_type")
    
  def test_setName(self):
    self.sdosp.setName("test")
    self.assertEqual(self.sdosp.getName(),"test")


  def test_setInterfaceType(self):
    self.sdosp.setInterfaceType("test_type")
    self.assertEqual(self.sdosp.getInterfaceType(),"test_type")
    

  def test_setIdlDefinition(self):
    self.sdosp.setIdlDefinition("test_idl")
    self.assertEqual(self.sdosp.getIdlDefinition(),"test_idl")


  def test_setProperties(self):
    self.sdosp.setProperties(None)
    self.assertEqual(self.sdosp.getProperties(),None)


  def test_setServiceRef(self):
    self.sdosp.setServiceRef(None)
    self.assertEqual(self.sdosp.getServiceRef(),None)

  
############### test #################
if __name__ == '__main__':
        unittest.main()
