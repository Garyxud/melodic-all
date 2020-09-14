#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_OutPortConsumer.py
#  \brief test for OutPortConsumer class
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
from OutPortConsumer import *

import RTC, RTC__POA

import OpenRTM_aist


class ConsumerMock:
	def subscribeInterface(self, prop):
		print "subscribeInterface"
		return

	def unsubscribeInterface(self, prop):
		print "unsubscribeInterface"
		return


class TestOutPortConsumer(unittest.TestCase):
	def setUp(self):
		self._oc = OutPortConsumerFactory.instance()
		return

	def test_OutPortConsumerFactory(self):
		_oc1 = OutPortConsumerFactory.instance()
		self.assertEqual(self._oc, _oc1)
		return

	def test_subscribe(self):
		subs = self._oc.subscribe(None)
		subs(ConsumerMock())
		subs(ConsumerMock())
		subs(ConsumerMock())
		return

	def test_unsubscribe(self):
		subs = self._oc.unsubscribe(None)
		subs(ConsumerMock())
		subs(ConsumerMock())
		subs(ConsumerMock())
		return


############### test #################
if __name__ == '__main__':
        unittest.main()
