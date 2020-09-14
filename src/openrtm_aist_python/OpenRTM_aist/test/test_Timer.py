#!/usr/bin/env/python
# -*- Python -*-

#
# \file Timer.py
# \brief Timer class
# \date $Date: $
# \author Noriaki Ando <n-ando@aist.go.jp>
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
import OpenRTM_aist

from Timer import *


class test:
  def func(self):
    print "test.hello."

  def invoke(self):
    print "test invoke"
    
class TestTimer(unittest.TestCase):
  def setUp(self):
    self.tm = Timer(OpenRTM_aist.TimeValue())
    
  def tearDown(self):
    self.tm.__del__()
    OpenRTM_aist.Manager.instance().shutdownManager()
    time.sleep(0.1)

  def test_start_stop(self):
    self.tm.start()
    self.tm.stop()
    

  def test_invoke(self):
    self.tm.registerListenerFunc(test().func, OpenRTM_aist.TimeValue())
    self.tm.registerListenerFunc(test().func, OpenRTM_aist.TimeValue())
    self.tm.invoke()
    

  def test_registerListener(self):
    self.tm.registerListener(test(), OpenRTM_aist.TimeValue())
    self.tm.invoke()
    pass

    
  def test_registerListenerObj(self):
    self.tm.registerListenerObj(test(), test.func, OpenRTM_aist.TimeValue())
    self.tm.invoke()
    

  def test_registerListenerFunc(self):
    self.tm.registerListenerFunc(test().func, OpenRTM_aist.TimeValue())
    self.tm.invoke()


  def test_unregisterListener(self):
    obj = OpenRTM_aist.ListenerObject(test(),test.func)
    self.tm.registerListener(obj, OpenRTM_aist.TimeValue())
    self.assertEqual(self.tm.unregisterListener(obj),True)
    self.assertEqual(self.tm.unregisterListener(obj),False)


############### test #################
if __name__ == '__main__':
  unittest.main()
