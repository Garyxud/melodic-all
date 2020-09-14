#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_DataInPort.py
#  \brief RTC::Port implementation for Data InPort
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

from DataInPort import *

class TestObj(OpenRTM_aist.RTObject_impl):
	def __init__(self):
		self._orb = CORBA.ORB_init()
		self._poa = self._orb.resolve_initial_references("RootPOA")
		OpenRTM_aist.RTObject_impl.__init__(self, orb=self._orb, poa=self._poa)


class TestDataInPort(unittest.TestCase):
	def setUp(self):
		self._orb = CORBA.ORB_init()
		self._poa = self._orb.resolve_initial_references("RootPOA")
		inport = OpenRTM_aist.InPort("in", RTC.TimedLong(RTC.Time(0,0),0),
								OpenRTM_aist.RingBuffer(8))
		self._dip = DataInPort("in", inport, None)
		
	def test_case(self):
		nvlist = [OpenRTM_aist.NVUtil.newNV("dataport.interface_type","CORBA_Any"),
				  OpenRTM_aist.NVUtil.newNV("dataport.dataflow_type","Push"),
				  OpenRTM_aist.NVUtil.newNV("dataport.subscription_type","Flush")]

		prof = RTC.ConnectorProfile("connector0","",[TestObj(),TestObj()],nvlist)

		self.assertEqual(self._dip.publishInterfaces(prof), RTC.RTC_OK)
		self.assertEqual(self._dip.subscribeInterfaces(prof), RTC.RTC_OK)	
		self._dip.unsubscribeInterfaces(prof)
		
    
############### test #################
if __name__ == '__main__':
        unittest.main()
