#!/usr/bin/env python
# -*- Python -*-

#
#  \file  test_OutPortCorbaCdrConsumer.py
#  \brief test for OutPortCorbaCdrConsumer class
#  \date  $Date: 2007/09/26 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Noriaki Ando
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
 
from omniORB import any
from omniORB import CORBA

import OpenRTM_aist
import RTC, RTC__POA 
import SDOPackage, SDOPackage__POA

import sys
sys.path.insert(1,"../")

import unittest

from OutPortCorbaCdrConsumer import *

class DummyBuffer:
	def empty():
		return False

	def read(cdr):
		cdr[0] = 123
		return 0

class TestOutPortCorbaCdrConsumer(unittest.TestCase):

	def setUp(self):
		OpenRTM_aist.Manager.instance()
		OpenRTM_aist.OutPortCorbaCdrProviderInit()
		self._opp = OpenRTM_aist.OutPortCorbaCdrProvider()
		self._opp.setBuffer(DummyBuffer())
		self._opcc = OutPortCorbaCdrConsumer()
		return
    
	def test_setBuffer(self):
		self._opcc.setBuffer(DummyBuffer())
		return

	def COMMENTtest_get(self):
		data = [None]
		self.assertEqual(self._opcc.subscribeInterface(self._opp._properties),True)
		self.assertEqual(self._opcc.get(data),OpenRTM_aist.DataPortStatus.PORT_OK)
		self.assertEqual(data[0],123)
		return

	def test_subscribeInterface(self):
		self.assertEqual(self._opcc.subscribeInterface(self._opp._properties),True)
		return

	def test_unsubscribeInterface(self):
		self._opcc.unsubscribeInterface(self._opp._properties)
		return


############### test #################
if __name__ == '__main__':
        unittest.main()
