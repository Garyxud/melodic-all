#!/usr/bin/env python
# -*- Python -*-

#
#  @file test_RTCUtil.py
#  @brief test for RTComponent utils
#  @date $Date: 2007/09/11 $
#  @author Shinji Kurihara
# 
#  Copyright (C) 2007
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.


import sys
sys.path.insert(1,"../")

import unittest
import OpenRTM_aist
import RTC, RTC__POA

from RTCUtil import *
from omniORB import CORBA

#class test(RTC__POA.DataFlowParticipant):
class test(OpenRTM_aist.RTObject_impl):
  def __init__(self):
    self.orb = CORBA.ORB_init()
    self.poa = self.orb.resolve_initial_references("RootPOA")
    OpenRTM_aist.RTObject_impl.__init__(self, orb=self.orb, poa=self.poa)
    poaManager = self.poa._get_the_POAManager()
    poaManager.activate()

  
class TestRTCUtil(unittest.TestCase):
  def setUp(self):
    pass

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_isDataFlowComponent(self):
    dfp_i = test()
    dfp = dfp_i._this()
    print isDataFlowComponent(dfp)

  def test_isFsmParticipant(self):
    dfp_i = test()
    dfp = dfp_i._this()
    print isFsmParticipant(dfp)


  def test_isFsmObject(self):
    dfp_i = test()
    dfp = dfp_i._this()
    print isFsmObject(dfp)


  def test_isMultiModeObject(self):
    dfp_i = test()
    dfp = dfp_i._this()
    print isMultiModeObject(dfp)



############### test #################
if __name__ == '__main__':
        unittest.main()
