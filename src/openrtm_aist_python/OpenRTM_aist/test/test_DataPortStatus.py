#!/usr/bin/env python
# -*- Python -*-

#
#  \file  test_DataPortStatus.py
#  \brief test for CorbaPort class
#  \date  $Date: 2007/09/27 $
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

import sys
sys.path.insert(1,"../")

import unittest

from DataPortStatus import *

	
class TestDataPortStatus(unittest.TestCase):
	def setUp(self):
		self._dps = DataPortStatus()


	def test_toString(self):
		self.assertEqual(self._dps.toString(DataPortStatus.PORT_OK), "PORT_OK")
		self.assertEqual(self._dps.toString(DataPortStatus.PORT_ERROR), "PORT_ERROR")
		self.assertEqual(self._dps.toString(DataPortStatus.BUFFER_FULL), "BUFFER_FULL")
		self.assertEqual(self._dps.toString(DataPortStatus.BUFFER_EMPTY), "BUFFER_EMPTY")
		self.assertEqual(self._dps.toString(DataPortStatus.BUFFER_TIMEOUT), "BUFFER_TIMEOUT")
		self.assertEqual(self._dps.toString(DataPortStatus.SEND_FULL), "SEND_FULL")
		self.assertEqual(self._dps.toString(DataPortStatus.SEND_TIMEOUT), "SEND_TIMEOUT")
		self.assertEqual(self._dps.toString(DataPortStatus.RECV_EMPTY), "RECV_EMPTY")
		self.assertEqual(self._dps.toString(DataPortStatus.RECV_TIMEOUT), "RECV_TIMEOUT")
		self.assertEqual(self._dps.toString(DataPortStatus.INVALID_ARGS), "INVALID_ARGS")
		self.assertEqual(self._dps.toString(DataPortStatus.PRECONDITION_NOT_MET), "PRECONDITION_NOT_MET")
		self.assertEqual(self._dps.toString(DataPortStatus.CONNECTION_LOST), "CONNECTION_LOST")
		self.assertEqual(self._dps.toString(DataPortStatus.UNKNOWN_ERROR), "UNKNOWN_ERROR")


############### test #################
if __name__ == '__main__':
        unittest.main()
