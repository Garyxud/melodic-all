#!/usr/bin/env python
# -*- Python -*-

#
#  \file  test_InPortProvider.py
#  \brief test for InPortProvider class
#  \date  $Date: 2007/09/20 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Noriaki Ando
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.


import sys
sys.path.insert(1,"../")

import unittest

from InPortProvider import *
import OpenRTM_aist


class TestInPortProvider(unittest.TestCase):
  def setUp(self):
    return

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_publishInterfaceProfile(self):
    InPortProviderFactory.instance().setInterfaceType("corba_cdr")
    InPortProviderFactory.instance().setDataFlowType("push,pull")
    InPortProviderFactory.instance().setSubscriptionType("flush,new,periodic")
    InPortProviderFactory.instance().publishInterfaceProfile([])

  def test_publishInterface(self):
    InPortProviderFactory.instance().setInterfaceType("corba_cdr")
    InPortProviderFactory.instance().setDataFlowType("push,pull")
    InPortProviderFactory.instance().setSubscriptionType("flush,new,periodic")
    self.assertEqual(InPortProviderFactory.instance().publishInterface([OpenRTM_aist.NVUtil.newNV("dataport.interface_type","corba_cdr")]),True)


############### test #################
if __name__ == '__main__':
        unittest.main()
