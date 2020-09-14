#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_OutPortConnector.py
#  \brief test for OutPortConnector class
#  \date $Date: 2007/09/19$
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Noriaki Ando
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 

import sys
sys.path.insert(1,"../")

from omniORB import *
from omniORB import any

import unittest
from OutPortConnector import *

import RTC, RTC__POA

import OpenRTM_aist



class TestOutPortConnector(unittest.TestCase):
	def setUp(self):
		self._profile = OpenRTM_aist.ConnectorBase.Profile("test",
								   "id",
								   ["in","out"],
								   OpenRTM_aist.Properties())

		self._oc = OutPortConnector(self._profile)
		return

	def test_profile(self):
		self.assertEqual(self._oc.profile(), self._profile)
		return


	def test_id(self):
		self.assertEqual(self._oc.id(), "id")
		return


	def test_name(self):
		self.assertEqual(self._oc.name(),"test")
		return


############### test #################
if __name__ == '__main__':
        unittest.main()
