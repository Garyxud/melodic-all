#!/usr/bin/env python
# -*- Python -*-

# \file test_CdrRingBuffer.py
# \brief test of CdrRingBuffer class
# \date $Date: 2007/09/12 $
# \author Shinji Kurihara
#
# Copyright (C) 2006
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
  
import sys
sys.path.insert(1,"../")

import unittest

from CdrBufferBase import *
from CdrRingBuffer import *

class TestCdrRingBuffer(unittest.TestCase):
	def setUp(self):
		CdrRingBufferInit()
		self._buff = CdrBufferFactory.instance().createObject("ring_buffer")


	def test_write(self):
		self.assertEqual(self._buff.write(100),0)


	def test_read(self):
		data = [0]
		self.assertEqual(self._buff.write(100),0)
		self.assertEqual(self._buff.read(data), 0)
		self.assertEqual(data[0], 100)


############### test #################
if __name__ == '__main__':
        unittest.main()
