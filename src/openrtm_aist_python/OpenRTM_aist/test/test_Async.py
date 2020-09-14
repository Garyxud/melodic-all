#!/usr/bin/env python
# -*- coding: euc-jp -*-

# @file test_Async.py
# @brief test for Async class
# @date $Date: 2009/02/18$
# @author Shinji Kurihara
#
# Copyright (C) 2009
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import time
import sys
sys.path.insert(1,"../")

import unittest
from Async import *

class A:
    	def __init__(self):
		self._hoge   = False
		self._munya  = False
		self._addone = False
		return

	def hoge(self):
		for i in range(5):
			print ",",
			time.sleep(1)
		print
		self._hoge = True
		return

	def hoge_invoked(self):
		return self._hoge

	def munya(self, msg):
		print
		for i in range(5):
			print ",",
			time.sleep(1)
		self._munya = True
		return

	def munya_invoked(self):
		return self._munya

	def add_one(self, val):
		for i in range(5):
			print ",",
			time.sleep(1)
		self._addone = True
		return val+1

	def add_one_invoked(self):
		return self._addone


class add_one_functor:
	def __init__(self, val):
		self._val = val
		self._ret = None

	def __call__(self, obj):
		self._ret = obj.add_one(self._val)
		
	def get_ret(self):
		return self._ret
		


class TestAsync(unittest.TestCase):
	def setUp(self):
		pass

	def test_memfun(self):
		a = A()
		invoker = Async_tInvoker(a,A.hoge)
		invoker.invoke()
		self.assertEqual(a.hoge_invoked(),False)
		invoker.wait()
		self.assertEqual(a.hoge_invoked(),True)
		return


	def test_memfun_with_args(self):
		a = A()
		invoker = Async_tInvoker(a,A.munya,"munyamunya")
		invoker.invoke()
		self.assertEqual(a.munya_invoked(),False)
		invoker.wait()
		self.assertEqual(a.munya_invoked(),True)
		return


	def test_functor(self):
		a = A()
		val = 100
		aof = add_one_functor(val)
		invoker = Async_tInvoker(a,aof)
		invoker.invoke()
		self.assertEqual(a.add_one_invoked(),False)
		invoker.wait()
		self.assertEqual(a.add_one_invoked(),True)
		self.assertEqual(aof.get_ret(),val+1)
		return

		

############### test #################
if __name__ == '__main__':
	unittest.main()
