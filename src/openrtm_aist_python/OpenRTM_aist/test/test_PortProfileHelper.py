#!/usr/bin/env python
# -*- Python -*-

#
#  \file test_PortProfileHelper.py
#  \brief test for RTC's PortProfileHelper class
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

import sys
sys.path.insert(1,"../")

import unittest
from PortProfileHelper import *

import CORBA
import OpenRTM_aist
import RTC, RTC__POA



class TestPortProfileHelper(unittest.TestCase):
	def setUp(self):
		return



############### test #################
if __name__ == '__main__':
        unittest.main()
