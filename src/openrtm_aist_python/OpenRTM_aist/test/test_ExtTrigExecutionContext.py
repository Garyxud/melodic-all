#!/usr/bin/env python
# -*- Python -*-

#
# \file test_ExtTrigExecutionContext.py
# \brief ExtTrigExecutionContext class
# \date $Date: 2007/09/06$
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
import threading

import unittest
import OpenRTM_aist
import RTC, RTC__POA

from ExtTrigExecutionContext import *

from omniORB import CORBA, PortableServer

class DFP(OpenRTM_aist.RTObject_impl):
  def __init__(self):
    self._orb = CORBA.ORB_init()
    self._poa = self._orb.resolve_initial_references("RootPOA")
    OpenRTM_aist.RTObject_impl.__init__(self, orb=self._orb, poa=self._poa)
    self._error = False
    self._ref = self._this()
    self._eclist = []

  def on_execute(self, ec_id):
    return RTC.RTC_OK

class TestExtTrigExecutionContext(unittest.TestCase):
  def setUp(self):
    self._dfp = DFP()
    self._dfp._poa._get_the_POAManager().activate()
    self.etec = ExtTrigExecutionContext()
    #self.etec = ExtTrigExecutionContext(self._dfp._ref)

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_tick(self):
    pass

  def test_run(self):
    self.assertEqual(self.etec.start(),RTC.RTC_OK)
    self.assertEqual(self.etec.add_component(self._dfp._this()),RTC.RTC_OK)
    self.assertEqual(self.etec.activate_component(self._dfp._this()),RTC.RTC_OK)
    import time
    time.sleep(1)
    self.etec.tick()
    self.etec.tick()
    time.sleep(1)
    self.assertEqual(self.etec.deactivate_component(self._dfp._this()),RTC.RTC_OK)
    time.sleep(1)
    self.assertEqual(self.etec.remove_component(self._dfp._this()),RTC.RTC_OK)
    th = threading.Thread(target=self.stop)
    th.start()
    self.etec.tick()
    if th:
      th.join()
    self._dfp._poa.deactivate_object(self._dfp._poa.servant_to_id(self.etec))
    self._dfp._poa.deactivate_object(self._dfp._poa.servant_to_id(self._dfp))

  def stop(self):
    self.etec.stop()

############### test #################
if __name__ == '__main__':
        unittest.main()
