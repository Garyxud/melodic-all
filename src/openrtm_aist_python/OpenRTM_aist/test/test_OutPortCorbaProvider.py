#!/usr/bin/env python
# -*- Python -*-

#
#  \file  test_OutPortCorbaProvider.py
#  \brief test for OutPortCorbaProvider class
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

import sys
sys.path.insert(1,"../")

import unittest

from OutPortCorbaProvider import *

import OpenRTM_aist
import RTC, RTC__POA

from omniORB import any
from omniORB import CORBA

class TestOutPortCorbaProvider(unittest.TestCase):
	def setUp(self):
		self.orb = CORBA.ORB_init()
		self.poa = self.orb.resolve_initial_references("RootPOA")
		poaManager = self.poa._get_the_POAManager()
		poaManager.activate()

		ringbuf = OpenRTM_aist.RingBuffer(8)
		ringbuf.init(RTC.TimedLong(RTC.Time(0,0), 0))
		outport = OpenRTM_aist.OutPort("out", RTC.TimedLong(RTC.Time(0,0), 123), ringbuf)
		self._opcp = OutPortCorbaProvider(outport)
		outport.write(RTC.TimedLong(RTC.Time(0,0), 123))


	def test_get(self):
		data = self._opcp.get()
		self.assertEqual(any.from_any(data, keep_structs=True).data, 123)


############### test #################
if __name__ == '__main__':
        unittest.main()
