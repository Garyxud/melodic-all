#!/usr/bin/env python
# -*- Python -*-


#  \file test_InPortCorbaCdrConsumer.py
#  \brief test for InPortCorbaCdrConsumer class
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

from InPortCorbaCdrConsumer import *

import RTC, RTC__POA
import OpenRTM_aist


class TestInPortCorbaCdrConsumer(unittest.TestCase):
	def setUp(self):
		InPortCorbaCdrConsumerInit()
		self._cons = OpenRTM_aist.InPortConsumerFactory.instance().createObject("corba_cdr")
		self._inp  = OpenRTM_aist.InPort("in",RTC.TimedLong(RTC.Time(0,0),0))
		self._orb  = OpenRTM_aist.Manager.instance().getORB()
		return
	
	def test_init(self):
		self._cons.init(OpenRTM_aist.Properties())
		return

	def test_put(self):
		self.assertEqual(self._cons.put(123),OpenRTM_aist.DataPortStatus.CONNECTION_LOST)
		return

	def test_publishInterfaceProfile(self):
		self._cons.publishInterfaceProfile(OpenRTM_aist.Properties())
		return

	def test_subscribeInterface(self):
		ior = self._orb.object_to_string(self._inp.get_port_profile().port_ref)
		self.assertEqual(self._cons.subscribeInterface([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ior",ior)]),True)
		self.assertEqual(self._cons.subscribeInterface([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ref",
											  self._inp.get_port_profile().port_ref)]),True)
		return

	def test_unsubscribeInterface(self):
		ior = self._orb.object_to_string(self._inp.get_port_profile().port_ref)
		self.assertEqual(self._cons.subscribeInterface([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ior",ior)]),True)
		self._cons.unsubscribeInterface([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ior",ior)])
		self.assertEqual(self._cons.subscribeInterface([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ref",
											  self._inp.get_port_profile().port_ref)]),True)
		self._cons.unsubscribeInterface([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ref",
									   self._inp.get_port_profile().port_ref)])
		return

	def test_subscribeFromIor(self):
		ior = self._orb.object_to_string(self._inp.get_port_profile().port_ref)
		self.assertEqual(self._cons.subscribeFromIor([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ior",ior)]),True)
		return

	def test_subscribeFromRef(self):
		self.assertEqual(self._cons.subscribeFromRef([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ref",
											self._inp.get_port_profile().port_ref)]),True)
		return

	def test_unsubscribeFromIor(self):
		ior = self._orb.object_to_string(self._inp.get_port_profile().port_ref)
		self.assertEqual(self._cons.subscribeFromIor([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ior",ior)]),True)
		self.assertEqual(self._cons.unsubscribeFromIor([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ior",ior)]),True)
		return

	def test_unsubscribeFromRef(self):
		self.assertEqual(self._cons.subscribeFromRef([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ref",
											self._inp.get_port_profile().port_ref)]),True)
		self.assertEqual(self._cons.unsubscribeFromRef([OpenRTM_aist.NVUtil.newNV("dataport.corba_cdr.inport_ref",
											  self._inp.get_port_profile().port_ref)]),True)
		return

############### test #################
if __name__ == '__main__':
        unittest.main()

