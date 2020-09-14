#!/usr/bin/env python
# -*- Python -*-

#
# \file test_Listener.py
# \brief test for Listener class
# \date $Date: 2007/08/23$
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

from Listener import *
import OpenRTM_aist

class test:
  def func(self):
    print "test func"


class TestListener(unittest.TestCase):
  def setUp(self):
    self.obj = ListenerObject(test(),test.func)
    self.func = ListenerFunc(test().func)


  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_invoke(self):
    self.obj.invoke()
    self.func.invoke()



############### test #################
if __name__ == '__main__':
        unittest.main()
