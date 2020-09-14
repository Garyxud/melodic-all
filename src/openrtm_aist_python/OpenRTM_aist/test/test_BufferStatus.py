#!/usr/bin/env python
# -*- Python -*-

# \file test_BufferStatus.py
# \brief test of BufferStatus class
# \date $Date: 2007/09/12 $
# \author Shinji Kurihara
#
# Copyright (C) 2009
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

from BufferStatus import *

class MyBuffer(BufferStatus):
	def __init__(self):
		pass

class TestBufferStatus(unittest.TestCase):
	def setUp(self):
		self.bf = MyBuffer()


	def test_toString(self):
		self.assertEqual(self.bf.toString(BufferStatus.BUFFER_OK),"BUFFER_OK")
		self.assertEqual(self.bf.toString(BufferStatus.BUFFER_ERROR),"BUFFER_ERROR")
		self.assertEqual(self.bf.toString(BufferStatus.BUFFER_FULL),"BUFFER_FULL")
		self.assertEqual(self.bf.toString(BufferStatus.BUFFER_EMPTY),"BUFFER_EMPTY")
		self.assertEqual(self.bf.toString(BufferStatus.NOT_SUPPORTED),"NOT_SUPPORTED")
		self.assertEqual(self.bf.toString(BufferStatus.TIMEOUT),"TIMEOUT")
		self.assertEqual(self.bf.toString(BufferStatus.PRECONDITION_NOT_MET),"PRECONDITION_NOT_MET")



############### test #################
if __name__ == '__main__':
        unittest.main()
