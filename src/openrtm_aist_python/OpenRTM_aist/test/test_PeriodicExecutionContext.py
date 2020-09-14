#!/usr/bin/env python
# -*- Python -*-

#
# \file test_PeriodicExecutionContext.py
# \brief test for PeriodicExecutionContext class
# \date $Date: 2007/08/28$
# \author Shinji Kurihara
#
# Copyright (C) 2006
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
sys.path.insert(1,"../")

import unittest
import OpenRTM_aist
from PeriodicExecutionContext import *

import RTC
import RTC__POA
import SDOPackage
from omniORB import CORBA, PortableServer


#class DFP(RTC__POA.DataFlowComponent):
class DFP(OpenRTM_aist.RTObject_impl):
  def __init__(self):
    self._orb = CORBA.ORB_init()
    self._poa = self._orb.resolve_initial_references("RootPOA")
    OpenRTM_aist.RTObject_impl.__init__(self, orb=self._orb, poa=self._poa)
    self._error = False
    self._ref = self._this()
    self._eclist = []

  def getRef(self):
    return self._ref

  def gotoError(self):
    self._error = True

  def initialize(self):
    return RTC.RTC_OK

  def finalize(self):
    return RTC.RTC_OK

  def exit(self):
    return RTC.RTC_OK

  def is_alive(self):
    return True

  def get_contexts(self):
    return self._eclist

  def set_execution_context_service(self, ec):
    self._eclist.append(ec)
    return len(self._eclist) -1 

  def get_component_profile(self):
    return None

  def get_ports(self):
    return None

  def get_execution_context_services(self):
    return None

  def on_initialize(self):
    print "on_initialize()"
    return RTC.RTC_OK

  def on_finalize(self):
    print "on_finalize()"
    return RTC.RTC_OK

  def on_startup(self, id):
    print "on_startup()"
    return RTC.RTC_OK

  def on_shutdown(self, id):
    print "shutdown()"
    return RTC.RTC_OK

  def on_activated(self, id):
    print "activated()"
    return RTC.RTC_OK

  def on_deactivated(self, id):
    print "deactivated()"
    return RTC.RTC_OK

  def on_aborting(self, id):
    print "on_aborting()"
    return RTC.RTC_OK
  
  def on_error(self, id):
    print "on_error()"
    return RTC.RTC_OK
  
  def on_reset(self, id):
    print "on_reset()"
    return RTC.RTC_OK

  def on_execute(self, id):
    print "on_execute()"
    if self._error:
      self._error = False
      return RTC.RTC_ERROR
    return RTC.RTC_OK
    
  def on_state_update(self, id):
    print "on_state_update()"
    return RTC.RTC_OK

  def on_rate_changed(self, id):
    print "on_rate_changed()"
    return RTC.RTC_OK

  def get_owned_organizations(self):
    return None

  def get_sdo_id(self):
    return None

  def get_sdo_type(self):
    return None

  def get_device_profile(self):
    return None

  def get_service_profiles(self):
    return None

  def get_service_profile(self, id):
    return None

  def get_sdo_service(self):
    return SDOPackage.SDOService._nil()

  def get_configuration(self):
    return SDOPackage.Configuration._nil()

  def get_monitoring(self):
    return SDOPackage.Monitoring._nil()

  def get_organizations(self):
    return None

  def get_status_list(self):
    return None

  def get_status(self, name):
    return None
  

class TestPeriodicExecutionContext(unittest.TestCase):
  def setUp(self):
    self._dfp = DFP()
    self._dfp._poa._get_the_POAManager().activate()

    self._pec = PeriodicExecutionContext(self._dfp._ref, 10)
    self._pec.add_component(self._dfp.getRef())
    self._pec.start()
  
  def tearDown(self):
    self._pec.stop()
    # import gc
    self._pec.__del__()
    self._pec = None
    OpenRTM_aist.Manager.instance().shutdownManager()

  def getState(self, state):
    if state == 0:
      return "INACTIVE_STATE"
    elif state == 1:
      return "ACTIVE_STATE"
    elif state == 2:
      return "ERROR_STATE"
    elif state == 3:
      return "UNKNOWN_STATE"

    return "INVALID_STATE"
    

  def test_case(self):
    print "rate: ", self._pec.get_rate()
    print "kind: ", self._pec.get_kind()
    self._pec.activate_component(self._dfp.getRef())
    #for i in range(3):
    # state = self._pec.get_component_state(self._dfp.getRef())
    # print self.getState(state)
    #self._pec.stop()
    #print "is_running : ", self._pec.is_running()
    import time
    time.sleep(1)

  
  def test_set_rate(self):
    self._pec.set_rate(1000)
    print "get rate: ", self._pec.get_rate()

  
  def test_activate_component(self):
    #self._pec.activate_component(self._dfp.getRef())
    #self._pec.deactivate_component(self._dfp.getRef())
    pass



  def test_reset_component(self):
    self._pec.reset_component(self._dfp.getRef())
    print "reset state: ", self._pec.get_component_state(self._dfp.getRef())
    self._pec.activate_component(self._dfp.getRef())
    print "activate state: ", self._pec.get_component_state(self._dfp.getRef())

  
  def test_remove(self):
    self._pec.remove_component(self._dfp.getRef())
    self._pec.activate_component(self._dfp.getRef())
    print "activate state: ", self._pec.get_component_state(self._dfp.getRef())


  def test_get_profile(self):
    print "get_profile.kind: ", self._pec.get_profile().kind

  

############### test #################
if __name__ == '__main__':
        unittest.main()
