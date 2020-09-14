#!/usr/bin/env python
# -*- Python -*-

# @file test_Singleton.py
# @brief test for Singleton class
# @date $Date$
# @author Shinji Kurihara
#
# Copyright (C) 2003-2005
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

#
# $Log$
#
#

import sys
sys.path.insert(1,"../")

import unittest

from Singleton import *

class SingletonMock(Singleton):
	_cnt = 0
	def __init__(self):
		print "SingletonMock"
		#self._cnt = 0
		pass

	def up(self):
		self._cnt += 1

	def get(self):
		return self._cnt


class TestSingleton(unittest.TestCase):
	def setUp(self):
		return

	def tearDown(self):
		return


	def test_SingletonCase(self):
		mock = SingletonMock()
		print "Singleton 0"
		mock0 = SingletonMock()
		print "mock0 ", mock0
		print "Singleton 1"
		mock1 = SingletonMock()
		print "mock1 ", mock1
		self.assertEqual(mock0.get(), 0)
		self.assertEqual(mock1.get(), 0)
		mock0.up()
		self.assertEqual(mock0.get(), 1)
		self.assertEqual(mock1.get(), 1)
		mock1.up()
		self.assertEqual(mock0.get(), 2)
		self.assertEqual(mock1.get(), 2)
		mock2 = SingletonMock.instance()
		print "mock2 ", mock2
		self.assertEqual(mock0.get(), 2)
		self.assertEqual(mock1.get(), 2)
		self.assertEqual(mock2.get(), 2)
		return

if __name__ == "__main__":
	unittest.main()
