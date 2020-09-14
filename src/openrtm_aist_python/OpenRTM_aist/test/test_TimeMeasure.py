#!/usr/bin/env python
# -*- coding: euc-jp -*-

# @file test_TimeMeasure.py
# @brief test for TimeMeasure class
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
import math
sys.path.insert(1,"../")

import unittest
from TimeMeasure import *



class TestTimeMeasure(unittest.TestCase):
  def setUp(self):
    self._tm = TimeMeasure(10)
  
  def test_tick_tack(self):
    for i in range(10):
      self._tm.tick()
      time.sleep(0.01)
      self._tm.tack()
    _max    = [0]
    _min    = [0]
    _mean   = [0]
    _stddev = [0]
    print "count: ", self._tm.count()
    print "result: ", self._tm.getStatistics(_max, _min, _mean, _stddev)
    print "max: ", _max[0]
    print "min: ", _min[0]
    print "mean: ", _mean[0]
    print "stddev: ", _stddev[0]
    return

  def test_coutn(self):
    tm_ = TimeMeasure()
    self.assertEqual(0, tm_.count())

    count0 = 10
    for i in range(10):
      tm_.tick()
      tm_.tack()
    self.assertEqual(10, tm_.count())

    tm_.reset()
    self.assertEqual(0, tm_.count())


    count0 = 102
    for i in range(102):
      tm_.tick()
      tm_.tack()
    self.assertNotEqual(102, tm_.count())
    return

  def test_stat(self):
    tm_ = TimeMeasure()
    maxi  = [0.0]
    mini  = [0.0]
    mean  = [0.0]
    stdev = [0.0]

    self.assertEqual(False, tm_.getStatistics(maxi, mini, mean, stdev))
    tm_.tick()
    self.assertEqual(False, tm_.getStatistics(maxi, mini, mean, stdev))
    tm_.tack()
    self.assertEqual(True, tm_.getStatistics(maxi, mini, mean, stdev))
    tm_.reset()
    self.assertEqual(False, tm_.getStatistics(maxi, mini, mean, stdev))
    return

  def test_buflen(self):
    tm_0 = TimeMeasure(1)
    self.assertEqual(0, tm_0.count())
    tm_0.tick()
    tm_0.tack()
    self.assertEqual(1, tm_0.count())

    count_ = 1024
    tm_1 = TimeMeasure(count_)

    for i in range(count_):
      self.assertEqual(i, tm_1.count())
      tm_1.tick()
      tm_1.tack()

    for i in range(count_):
      tm_1.tick()
      tm_1.tack()
      self.assertEqual(count_ + 1, tm_1.count())
    return

  def test_30ms(self):
    wait_ = 0.03 # [s]
    tm_ = TimeMeasure()
    for i in range(10):
      tm_.tick()
      time.sleep(wait_)
      tm_.tack()

    maxi  = [0.0]
    mini  = [0.0]
    mean  = [0.0]
    stdev = [0.0]
    tm_.getStatistics(maxi, mini, mean, stdev)
    print "test_30ms"
    print "max interval : ", maxi[0], " [sec]"
    print "min interval : ", mini[0], " [sec]"
    print "mean interval: ", mean[0], " [sec]"
    print "stddev       : ", stdev[0], " [sec]"
    self.assert_(maxi[0] < (wait_ + 0.030))
    self.assert_(mini[0] > (wait_ - 0.010))
    self.assert_(math.fabs(mean[0] - wait_) < 0.03)
    self.assert_(stdev[0] < (wait_ / 5.0))
    return

  def test_1s(self):
    wait_ = 1.0 # [s]
    tm_ = TimeMeasure()
    for i in range(1):
      tm_.tick()
      time.sleep(wait_)
      tm_.tack()

    maxi  = [0.0]
    mini  = [0.0]
    mean  = [0.0]
    stdev = [0.0]
    tm_.getStatistics(maxi, mini, mean, stdev)
    print "test_1ms"
    print "max interval : ", maxi[0], " [sec]"
    print "min interval : ", mini[0], " [sec]"
    print "mean interval: ", mean[0], " [sec]"
    print "stddev       : ", stdev[0], " [sec]"
    self.assert_(maxi[0] < (wait_ + 0.030))
    self.assert_(mini[0] > (wait_ - 0.010))
    self.assert_(math.fabs(mean[0] - wait_) < 0.03)
    self.assert_(stdev[0] < (wait_ / 5.0))
    return
  
############### test #################
if __name__ == '__main__':
  unittest.main()
