#!/usr/bin/env/python
# -*- Python -*-

#
# \file Task.py
# \brief Task class
# \date $Date: $
# \author Shinji Kurihara
#
# Copyright (C) 2007
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys,time
sys.path.insert(1,"../")

import unittest

from Task import *

cnt = 0

class Mock(Task):
	def __init__(self):
		global cnt
		cnt += 1
		self.cnt = cnt
		Task.__init__(self)
		self.flag = False

	def svc(self):
		print "cnt: ",
		while self.flag:
			print self.cnt, " ",
			time.sleep(0.1)
		return

	def activate(self):
		self.flag = True
		Task.activate(self)
		return

	def finalize(self):
		self.flag = False

class Mock2(Task):
	def __init__(self,func):
		global cnt
		cnt += 1
		self.cnt = cnt
		Task.__init__(self)
		self.flag = False
		self.func = func

	def svc(self):
		time.sleep(1)
		self.func()
		return

	def activate(self):
		self.flag = True
		Task.activate(self)
		return

	def finalize(self):
		self.flag = False
		

class TestTask(unittest.TestCase):
	def setUp(self):
		self.mock = Mock()
		return

	def test_activate(self):
		self.mock.activate()
		mock2= Mock()
		mock2.activate()
		time.sleep(2)
		mock2.finalize()
		self.mock.finalize()
		return

	def test_wait(self):
		self.mock.activate()
		time.sleep(0.5)
		mock2= Mock2(self.mock.finalize)
		mock2.activate()
		self.mock.wait()
		mock2.finalize()
		return
    

	def test_reset(self):
		self.mock.reset()
		return


	def test_finalize(self):
		self.mock.finalize()
		return
    

	def test_svn_run(self):
		self.mock.svc_run()
		return


############### test #################
if __name__ == '__main__':
        unittest.main()
