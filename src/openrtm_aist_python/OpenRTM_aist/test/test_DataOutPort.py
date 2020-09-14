#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_DataOutPort.py
#  \brief test for Base class of OutPort
#  \date $Date: 2007/09/27 $
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
import RTC, RTC__POA

import CORBA
import sys
sys.path.insert(1,"../")

import unittest

from DataOutPort import *

class TestObj(OpenRTM_aist.RTObject_impl):
	def __init__(self):
		self._orb = CORBA.ORB_init()
		self._poa = self._orb.resolve_initial_references("RootPOA")
		OpenRTM_aist.RTObject_impl.__init__(self, orb=self._orb, poa=self._poa)

class TestDataOutPort(unittest.TestCase):
	def setUp(self):
		self._orb = CORBA.ORB_init()
		self._poa = self._orb.resolve_initial_references("RootPOA")
		outport = OpenRTM_aist.OutPort("out", RTC.TimedLong(RTC.Time(0,0),0),
								OpenRTM_aist.RingBuffer(8))
		self._dop = DataOutPort("out", outport, None)


	def test_case(self):
		nvlist = [OpenRTM_aist.NVUtil.newNV("dataport.interface_type","CORBA_Any"),
				  OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type","Push"),
				  OpenRTM_aist.NVUtil.newNV("dataport.subscription_type","Flush")]

		prof = RTC.ConnectorProfile("connector0","",[TestObj(),TestObj()],nvlist)

		self.assertEqual(self._dop.publishInterfaces(prof), RTC.RTC_OK)
		self.assertEqual(self._dop.subscribeInterfaces(prof), RTC.RTC_OK)	
		self._dop.unsubscribeInterfaces(prof)



############### test #################
if __name__ == '__main__':
        unittest.main()
