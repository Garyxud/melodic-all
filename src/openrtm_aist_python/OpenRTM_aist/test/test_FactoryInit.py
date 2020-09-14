#!/usr/bin/env python
# -*- Python -*-

#
# \file test_FactoryInit.py
# \brief test for RTComponent factory class
# \date $Date: $
# \author Shinji Kurihara
#
# Copyright (C) 2003-2005
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
sys.path.insert(1,"../")

import unittest

from FactoryInit import *

class TestFactoryInit(unittest.TestCase):

	def setUp(self):
		return

	def tearDown(self):
		return

	def test_FactoryInit(self):
		FactoryInit()
	

############### test #################
if __name__ == '__main__':
        unittest.main()
