#!/usr/bin/env python
# -#- Python -#-
#
# \file test_TimeValue.py
# \brief test for TimeValue class
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

import sys
sys.path.insert(1,"../")

import unittest
import time

from TimeValue import *
import OpenRTM_aist

class TestTimeValue(unittest.TestCase):
  
  def setUp(self):
    self.tm = TimeValue(usec=1000000)


  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    pass


  def test_set_time(self):
    tm = time.time()
    ret = self.tm.set_time(tm)


  def test_toDouble(self):
    self.test_set_time()
    print  self.tm.toDouble()

  def test_sec_usec(self):
    tm = TimeValue(usec=1000000)
    self.assertEqual(tm.sec(),1)
    self.assertEqual(tm.usec(),0)

  def test_normalize(self):
    tm = TimeValue(sec=0,usec=10000000)
    tm.normalize()
    self.assertEqual(tm.sec(),10)
    self.assertEqual(tm.usec(),0)

    tm = TimeValue(sec=1,usec=1000000)
    tm.normalize()
    self.assertEqual(tm.sec(),2)
    self.assertEqual(tm.usec(),0)

    tm = TimeValue(sec=1,usec=-100000)
    tm.normalize()
    self.assertEqual(tm.sec(),0)
    self.assertEqual(tm.usec(),900000)

  def test__str__(self):
    self.test_set_time()
    print self.tm
  
############### test #################
if __name__ == '__main__':
        unittest.main()
