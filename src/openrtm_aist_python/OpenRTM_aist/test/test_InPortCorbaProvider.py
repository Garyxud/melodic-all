#!/usr/bin/env python
# -*- Python -*-


#
#  \file  InPortCorbaProvider.py
#  \brief InPortCorbaProvider class
#  \date  $Date: 2007/09/25 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Noriaki Ando
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
 
import sys
sys.path.insert(1,"../")

import unittest

from InPortCorbaProvider import *
 
from omniORB import CORBA
import RTC, RTC__POA
import SDOPackage,SDOPackage__POA
from omniORB import any

import OpenRTM_aist


class TestInPortCorbaProvider(unittest.TestCase):
	def setUp(self):
		self.orb = CORBA.ORB_init()
		self.poa = self.orb.resolve_initial_references("RootPOA")
		poaManager = self.poa._get_the_POAManager()
		poaManager.activate()

		ringbuf = OpenRTM_aist.RingBuffer(8)
		ringbuf.init(RTC.TimedLong(RTC.Time(0,0), 0))
		self._ipcp = InPortCorbaProvider(OpenRTM_aist.InPort("in",
														 RTC.TimedLong(RTC.Time(0,0), 0),
														 ringbuf))

	def test_publishInterface(self):
		prop = [SDOPackage.NameValue("dataport.interface_type",	"CORBA_Any")]
		self._ipcp.publishInterface(prop)


	def test_put(self):
		data = any.to_any(RTC.TimedLong(RTC.Time(0,0),123))
		self._ipcp.put(data)


############### test #################
if __name__ == '__main__':
        unittest.main()
