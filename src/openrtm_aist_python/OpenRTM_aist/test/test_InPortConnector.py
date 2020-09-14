#!/usr/bin/env python
# -*- Python -*-


#  \file test_InPortConnector.py
#  \brief test for InPortConnector class
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

from InPortConnector import *

import RTC, RTC__POA
import OpenRTM_aist


class TestInPortConnector(unittest.TestCase):
	def setUp(self):
		self._prof = OpenRTM_aist.ConnectorBase.Profile("name","id",[],OpenRTM_aist.Properties())
		self._ic = InPortConnector(self._prof,None)
		return
	
	def test_profile(self):
		self.assertEqual(self._ic.profile(),self._prof)
		return

	def test_id(self):
		self.assertEqual(self._ic.id(),"id")
		return

	def test_name(self):
		self.assertEqual(self._ic.name(), "name")

	def test_getBuffer(self):
		self.assertEqual(self._ic.getBuffer(),None)



############### test #################
if __name__ == '__main__':
        unittest.main()

