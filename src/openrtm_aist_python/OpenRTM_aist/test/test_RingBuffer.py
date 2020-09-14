#!/usr/bin/env python
# -*- Python -*-

# \file test_RingBuffer.py
# \brief test for Defautl Buffer class
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
sys.path.insert(1,"../RTM_IDL")

import unittest

from RingBuffer import *
import OpenRTM_aist

class TestRingBuffer(unittest.TestCase):

  def setUp(self):
    self._rb = RingBuffer()

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_init(self):
    prop = OpenRTM_aist.Properties()
    prop.setProperty("length","5")
    prop.setProperty("write.full_policy","overwrite")
    #prop.setProperty("write.full_policy","do_nothing")
    #prop.setProperty("write.full_policy","block")
    prop.setProperty("write.timeout","5.0")
    prop.setProperty("read.full_policy","overwrite")
    #prop.setProperty("read.full_policy","do_nothing")
    #prop.setProperty("read.full_policy","block")
    prop.setProperty("read.timeout","5.0")
    self._rb.init(prop)
    self.assertEqual(self._rb.length(),5)
    return

  def test_length(self):
    self.assertEqual(self._rb.length(), 8)
    self.assertEqual(self._rb.length(7), OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(self._rb.length(), 7)
    self.assertEqual(self._rb.length(0), OpenRTM_aist.BufferStatus.NOT_SUPPORTED)
    self.assertEqual(self._rb.length(-1), OpenRTM_aist.BufferStatus.NOT_SUPPORTED)
    self.assertEqual(self._rb.length(1), OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(self._rb.length(), 1)
    return


  def test_reset(self):
    prop = OpenRTM_aist.Properties()
    prop.setProperty("length","5")
    prop.setProperty("write.full_policy","overwrite")
    #prop.setProperty("write.full_policy","do_nothing")
    #prop.setProperty("write.full_policy","block")
    prop.setProperty("write.timeout","5.0")
    prop.setProperty("read.full_policy","overwrite")
    #prop.setProperty("read.full_policy","do_nothing")
    #prop.setProperty("read.full_policy","block")
    prop.setProperty("read.timeout","5.0")
    self._rb.init(prop)
    self.assertEqual(self._rb.write(123), OpenRTM_aist.BufferStatus.BUFFER_OK)
    value = [None]
    self.assertEqual(self._rb.read(value),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(value[0],123)
    self._rb.reset()
    self.assertEqual(self._rb.read(value),OpenRTM_aist.BufferStatus.BUFFER_EMPTY)
    self.assertEqual(value[0],123)

    return

  def test_write(self):
    data=[0]
    self.assertEqual(self._rb.write(1),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self._rb.read(data)
    self.assertEqual(data[0],1)

    self.assertEqual(self._rb.write(2),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self._rb.read(data)
    self.assertEqual(data[0],2)

    self.assertEqual(self._rb.write(3),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self._rb.read(data)
    self.assertEqual(data[0],3)

    self.assertEqual(self._rb.write(4),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self._rb.read(data)
    self.assertEqual(data[0],4)

    self.assertEqual(self._rb.write(5),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self._rb.read(data)
    self.assertEqual(data[0],5)

    self.assertEqual(self._rb.write(6),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self._rb.read(data)
    self.assertEqual(data[0],6)

    self.assertEqual(self._rb.write("string"),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self._rb.read(data)
    self.assertEqual(data[0],"string")

    self.assertEqual(self._rb.write([1,2,3]),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self._rb.read(data)
    self.assertEqual(data[0],[1,2,3])

    self.assertEqual(self._rb.write(0.12345),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self._rb.read(data)
    self.assertEqual(data[0],0.12345)

    for i in range(8):
      self.assertEqual(self._rb.write(0.12345,1,0),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(self._rb.write(0.12345,1,0),OpenRTM_aist.BufferStatus.TIMEOUT)

  def test_read(self):
    data=[0]
    self.assertEqual(self._rb.read(data,1,0),OpenRTM_aist.BufferStatus.TIMEOUT)
    self.assertEqual(self._rb.write("string"),OpenRTM_aist.BufferStatus.BUFFER_OK)
    # Failure pattern (parameter must be List object.)
    # data=0
    # self._rb.read(data)
    self.assertEqual(self._rb.read(data),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(data[0],"string")
    self.assertEqual(self._rb.read(data,1,0),OpenRTM_aist.BufferStatus.TIMEOUT)

  def test_readable(self):
    data=[0]
    self.assertEqual(self._rb.readable(),0)
    self.assertEqual(self._rb.write("string"),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(self._rb.readable(),1)
    self.assertEqual(self._rb.read(data),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(self._rb.readable(),0)
    self._rb.read(data)
    self.assertEqual(self._rb.readable(),0)
    
  def test_empty(self):
    data=[0]
    self.assertEqual(self._rb.empty(),True)
    self.assertEqual(self._rb.write("string"),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(self._rb.empty(),False)
    self.assertEqual(self._rb.read(data),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(self._rb.empty(),True)
    self._rb.read(data)
    self.assertEqual(self._rb.empty(),True)
    
  def COMMENTtest_isFull(self):
    self.assertEqual(self._rb.isFull(),False)


  def COMMENTtest_isEmpty(self):
    self.assertEqual(self._rb.isEmpty(),True)
    self._rb.init(0)
    self.assertEqual(self._rb.isEmpty(),False)


  def COMMENTtest_isNew(self):
    self.assertEqual(self._rb.isNew(),False)
    self._rb.init(0)
    self.assertEqual(self._rb.isNew(),True)
    data=[0]
    self._rb.read(data)
    self.assertEqual(self._rb.isNew(),False)

    self.assertEqual(self._rb.write(0.12345),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(self._rb.write(0.12345),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(self._rb.write(0.12345),OpenRTM_aist.BufferStatus.BUFFER_OK)
    self.assertEqual(self._rb.isNew(),True)
    self.assertEqual(self._rb.isNew(),True)
    self.assertEqual(self._rb.isNew(),True)


############### test #################
if __name__ == '__main__':
        unittest.main()
