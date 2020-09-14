#!/usr/bin/env python
# -*- Python -*-


#  \file test_InPortConsumer.py
#  \brief test for InPort template class
#  \date $Date: 2007/09/20 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2003-2005
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
 

import sys
sys.path.insert(1,"../")

import unittest

from InPortConsumer import *

import RTC, RTC__POA
import OpenRTM_aist

class InPortConsumerMock:
	def __init__(self):
		pass

	def publishInterfaceProfile(self, prop):
		return True

	def subscribeInterface(self, prop):
		return True


class TestInPortConsumer(unittest.TestCase):
	def setUp(self):
		self._mock = InPortConsumerMock()
		return
	
	def test_publishInterfaceProfileFunc(self):
		pubFunc = InPortConsumerFactory.instance().publishInterfaceProfileFunc(None)
		pubFunc(self._mock)
		return

	def test_subscribeInterfaceFunc(self):
		subFunc = InPortConsumerFactory.instance().subscribeInterfaceFunc(None)
		self.assertEqual(subFunc(self._mock),True)
		return


############### test #################
if __name__ == '__main__':
        unittest.main()

