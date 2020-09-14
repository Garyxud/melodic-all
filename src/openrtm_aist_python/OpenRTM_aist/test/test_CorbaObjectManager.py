#!/usr/bin/env python
# -*- Python -*-

#
# \file test_CorbaObjManager.py
# \brief test for CORBA Object manager class
# \date $Date: 2007/08/27$
# \author Shinji Kurihara
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

import OpenRTM_aist

from omniORB import CORBA, PortableServer
from CorbaObjectManager import *



class test_comp(OpenRTM_aist.RTObject_impl):
	def __init__(self):
		pass
		
	def echo(self, msg):
		print msg
		return msg
		

class TestCorbaObjectManager(unittest.TestCase):

	def setUp(self):
		self._orb = CORBA.ORB_init()
		self._poa = self._orb.resolve_initial_references("RootPOA")
		self._poa._get_the_POAManager().activate()

		self._com = CorbaObjectManager(self._orb, self._poa)

		self._obj = test_comp()

	def tearDown(self):
		pass


	def test_activate(self):
		self._com.activate(self._obj)

	def test_deactivate(self):
		self._com.deactivate(self._obj)

############### test #################
if __name__ == '__main__':
        unittest.main()
