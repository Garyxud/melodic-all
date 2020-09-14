#!/usr/bin/env python
# -*- Python -*-

#
# \file test_NamingManager.py
# \brief test for naming Service helper class
# \date $Date: 2007/08/27$
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
from omniORB import CORBA

import unittest

import OpenRTM_aist

from NamingManager import *


class test_comp(OpenRTM_aist.RTObject_impl):
	def __init__(self):
		pass
		
	def echo(self, msg):
		print msg
		return msg

class TestNamingManager(unittest.TestCase):

	def setUp(self):
		self._mgr = OpenRTM_aist.Manager.init(sys.argv)
		self._nm  = NamingManager(self._mgr)
		self._obj = test_comp()
		self._mgrservant = OpenRTM_aist.ManagerServant()
	def __del__(self):
		pass

	def test_bindObject(self):
		self._noc.bindObject("test_comp",self._obj)
		return

	def test_unbindObject(self):
		self._noc.unbindObject("test_comp")
		return


	def test_registerNameServer(self):
		self._nm.registerNameServer("test_comp","localhost")
		return

	def test_bindObject(self):
		self._nm.bindObject("test_comp",self._obj)
		self._nm.registerNameServer("test_comp","localhost")
		self._nm.bindObject("test_comp",self._obj)
		return

	def test_bindManagerObject(self):
		self._nm.bindManagerObject("test_mgr",self._mgrservant)
		self._nm.registerNameServer("test_comp","localhost")
		self._nm.bindManagerObject("test_mgr",self._mgrservant)
		
	def test_update(self):
		self._nm.update()
		self._nm.registerNameServer("test_comp","localhost")
		self._nm.update()
		return
	
	def test_unbindObject(self):
		self._nm.unbindObject("test_comp")
		self._nm.registerNameServer("test_comp","localhost")
		self._nm.unbindObject("test_comp")
		return

	def test_unbindAll(self):
		self._nm.unbindAll()
		self._nm.registerCompName("rest",self._obj)
		self._nm.unbindAll()
		self._nm.registerMgrName("rest",self._mgrservant)
		self._nm.unbindAll()
		return

	def test_getObjects(self):
		self._nm.bindObject("test_comp",self._obj)
		self.assertEqual(len(self._nm.getObjects()),1)
		return

	def test_createNamingObj(self):
		self._nm.createNamingObj("test", "localhost")
		return

	def test_bindCompsTo(self):
		self._nm.bindCompsTo(self._obj)
		return

	def test_registerCompName(self):
		self._nm.registerCompName("rest",self._obj)
		return

	def test_registerMgrName(self):
		self._nm.registerMgrName("rest",self._mgrservant)
		return

	def test_unregisterCompName(self):
		self._nm.registerCompName("rest",self._obj)
		self._nm.unregisterCompName("rest")
		return

	def test_unregisterMgrName(self):
		self._nm.registerMgrName("rest",self._mgrservant)
		self._nm.unregisterMgrName("rest")
		return


############### test #################
if __name__ == '__main__':
        unittest.main()
