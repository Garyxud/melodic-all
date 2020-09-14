#!/usr/bin/env python
# -*- Python -*-

#
#  \file  test_OutPortCorbaConsumer.py
#  \brief test for OutPortCorbaConsumer class
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

from OutPortCorbaConsumer import *

class OutPortTest(RTC__POA.OutPortAny):
	def __init__(self):
		self.orb = CORBA.ORB_init()
		self.poa = self.orb.resolve_initial_references("RootPOA")
		poaManager = self.poa._get_the_POAManager()
		poaManager.activate()
		
		
	def get(self):
		print "Called get operation."
		return any.to_any(RTC.TimedLong(RTC.Time(0,0),123))


class TestOutPortCorbaConsumer(unittest.TestCase):

	def setUp(self):
		ringbuf = OpenRTM_aist.RingBuffer(8)
		ringbuf.init(RTC.TimedLong(RTC.Time(0,0),0))
		self._opcc = OutPortCorbaConsumer(OpenRTM_aist.InPort("in",
														 RTC.TimedLong(RTC.Time(0,0),0),
														 ringbuf))
		

    
	def test_get(self):
		self._opcc.setObject(OutPortTest()._this())
		data=[None]
		self.assertEqual(self._opcc.get(data),True)
		self.assertEqual(data[0].data,123)


	def test_pull(self):
		self._opcc.pull()


	def test_subscribeInterface(self):
		port = any.to_any(OutPortTest()._this())
		prop = [SDOPackage.NameValue("dataport.dataflow_type","Push"),
				SDOPackage.NameValue("dataport.corba_any.outport_ref",port)]
		self.assertEqual(self._opcc.subscribeInterface(prop), True)
		self._opcc.unsubscribeInterface(prop)


############### test #################
if __name__ == '__main__':
        unittest.main()
