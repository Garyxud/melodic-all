#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_Guard.py
#  \brief test for Guard class
#  \date $Date: 2007/09/18 $
#  \author Shinji Kurihara
# 
#  Copyright (C) 2006
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 

import sys, time
sys.path.insert(1,"../")

import unittest
import threading
from Guard import *
from Task import *

ShareCount = 0

class TestGuardTask(Task):
	def __init__(self):
		global ShareCount
		self.ShareCount = 1
		Task.__init__(self)
		self.flag = False
		self._mutex = threading.RLock()

	def svc(self):
		global ShareCount
		guard = ScopedLock(self._mutex)
		for i in range(100):
			ShareCount += 1
		time.sleep(0.1)
		self.ShareCount = ShareCount
		return

	def getShareCount(self):
		return self.ShareCount

class TestGuard(unittest.TestCase):
	def setUp(self):
		return

	def test_case0(self):
		task1 = TestGuardTask()
		task2 = TestGuardTask()
		task1.activate()
		task2.activate()
		task1.wait()
		task2.wait()
		count = task1.getShareCount()
		self.assertEqual(count, 200)
		count = task2.getShareCount()
		self.assertEqual(count, 200)
		task2.finalize()
		task1.finalize()
		return



############### test #################
if __name__ == '__main__':
        unittest.main()
