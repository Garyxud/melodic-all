#!/usr/bin/env python
# -*- Python -*-

#
#  \file  test_InPortCorbaConsumer.py
#  \brief test for InPortCorbaConsumer class
#  \date  $Date: 2007/09/21 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Noriaki Ando
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
 

import OpenRTM_aist

import sys
sys.path.insert(1,"../")

import unittest

from InPortCorbaConsumer import *
from NVUtil import *
 
from omniORB import CORBA
import RTC, RTC__POA
import SDOPackage,SDOPackage__POA
from omniORB import any


class InPortTest(RTC__POA.InPortAny):
	def __init__(self):
		self.orb = CORBA.ORB_init()
		self.poa = self.orb.resolve_initial_references("RootPOA")
		poaManager = self.poa._get_the_POAManager()
		poaManager.activate()
		
		
	def put(self, data):
		print "put data: ", data

		
class test(OpenRTM_aist.RTObject_impl):
	def __init__(self):
		self.orb = CORBA.ORB_init()
		self.poa = self.orb.resolve_initial_references("RootPOA")
		OpenRTM_aist.RTObject_impl.__init__(self, orb=self.orb, poa=self.poa)
		poaManager = self.poa._get_the_POAManager()
		poaManager.activate()


class TestInPortCorbaConsumer(unittest.TestCase):
	def setUp(self):
		ringbuf = OpenRTM_aist.RingBuffer(8)
		ringbuf.init(RTC.TimedLong(RTC.Time(0,0), 0))
		self._ipcc = InPortCorbaConsumer(OpenRTM_aist.OutPort("out",
														 RTC.TimedLong(RTC.Time(0,0), 0),
														 ringbuf))
		
	def test_equal_operator(self):
		self.assertEqual(self._ipcc.equal_operator(self._ipcc), self._ipcc)

	def test_put(self):
		self._ipcc.setObject(InPortTest()._this())
		self._ipcc.put(RTC.TimedLong(RTC.Time(0,0), 123))

	def test_push(self):
		self._ipcc.setObject(InPortTest()._this())
		self._ipcc.push()

	def test_clone(self):
		self._ipcc.clone()

	def test_subscribeInterface(self):
		port = any.to_any(InPortTest()._this())
#		prop = [SDOPackage.NameValue("dataport.dataflow_type","Push"),
#				SDOPackage.NameValue("dataport.corba_any.inport_ref",port)]
		prop = [newNV("dataport.dataflow_type","Push"),
							newNV("dataport.corba_any.inport_ref",port)]
		self.assertEqual(self._ipcc.subscribeInterface(prop), True)

		
	def unsubscribeInterface(self, properties):
		pass


############### test #################
if __name__ == '__main__':
        unittest.main()
