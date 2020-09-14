#!/usr/bin/env python
# -*- Python -*-

#
# \file test_StateMachine.py
# \brief State machine template class
# \date $Date: 2007/08/30$
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

from StateMachine import *
import OpenRTM_aist

    

class TestStateMachine(unittest.TestCase):
# state = [0,1,2]
  state = [RTC.INACTIVE_STATE, RTC.ACTIVE_STATE, RTC.ERROR_STATE]

  def setUp(self):
    self._sm = StateMachine(3)

    self._sm.setNOP(self.nullAction)
    
    self._sm.setEntryAction(RTC.ACTIVE_STATE, self.on_activated)
    self._sm.setEntryAction(RTC.ERROR_STATE,self.on_aborting)
    self._sm.setPreDoAction(RTC.ACTIVE_STATE, self.on_reset)
    self._sm.setDoAction(RTC.ACTIVE_STATE, self.on_execute)
    self._sm.setPostDoAction(RTC.ACTIVE_STATE, self.on_state_update)
    self._sm.setExitAction(RTC.ACTIVE_STATE, self.on_deactivated)
    self._sm.setExitAction(RTC.ERROR_STATE, self.on_reset)
    self._sm.setTransitionAction(self.transition)

    self._sm.setListener(self)
    st = StateHolder()
    st.prev = RTC.INACTIVE_STATE
    st.curr = RTC.INACTIVE_STATE
    st.next = RTC.INACTIVE_STATE
    self._sm.setStartState(st)
    self._sm.goTo(RTC.INACTIVE_STATE)
    
  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def nullAction(self, st):
    print "nullAction."
    return True

  def on_activated(self, st):
    print "on_activated."
    return True

  def on_deactivated(self, st):
    print "on_deactivated."
    return True

  def on_aborting(self, st):
    print "on_aborting."
    return True

  def on_error(self, st):
    print "on_error."
    return True

  def on_reset(self, st):
    print "on_reset."
    return True

  def on_execute(self, st):
    print "on_execute."
    return True

  def on_state_update(self, st):
    print "on_state_update."
    return True

  def transition(self, st):
    print "transition."
    return True


  def test_setNOP(self):
    self._sm.setNOP(self.nullAction)
  
  
  def test_getStates(self):
    self.assertEqual(self._sm.getStates().curr, RTC.INACTIVE_STATE)
    self.assertEqual(self._sm.getStates().prev, RTC.INACTIVE_STATE)
    self.assertEqual(self._sm.getStates().next, RTC.INACTIVE_STATE)

    st = StateHolder()
    st.prev = RTC.ERROR_STATE
    st.curr = RTC.ERROR_STATE
    st.next = RTC.ERROR_STATE
    self._sm.setStartState(st)

    self.assertEqual(self._sm.getStates().curr, RTC.ERROR_STATE)
    self.assertEqual(self._sm.getStates().prev, RTC.ERROR_STATE)
    self.assertEqual(self._sm.getStates().next, RTC.ERROR_STATE)
    
  def test_getState(self):
    self.assertEqual(self._sm.getState(), RTC.INACTIVE_STATE)
  
  def test_isIn(self):
    self.assertEqual(self._sm.isIn(RTC.INACTIVE_STATE), True)
  
  def test_goTo(self):
    self._sm.goTo(RTC.INACTIVE_STATE)
  
#  def test_worker(self):
#   self._sm.goTo(RTC.ACTIVE_STATE)
#   self.assertEqual(self._sm.worker(), True)
#   self.assertEqual(self._sm.worker(), True)
#   self._sm.goTo(RTC.INACTIVE_STATE)
#   self.assertEqual(self._sm.worker(), True)
#   self.assertEqual(self._sm.worker(), True)
  

############### test #################
if __name__ == '__main__':
        unittest.main()
