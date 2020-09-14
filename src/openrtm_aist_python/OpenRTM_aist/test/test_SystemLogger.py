#!/usr/bin/env python
# -*- Python -*-

# @file test_SystemLogger.py
# @brief test for RT component logger class
# @date $Date$
# @author Shinji Kurihara
#
# Copyright (C) 2003-2005
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

#
# $Log$
#
#

import sys
sys.path.insert(1,"../")
import unittest

from SystemLogger import *
import OpenRTM_aist

i = 0

class TestLogger(unittest.TestCase):
  def setUp(self):

    #import random
    #val = random.uniform(0,100)
    #self.filename = "log" + str(val) + ".log"
    global i
    i+=1
    self.filename = "log" + str(i) + ".log"
    self.logstr = LogStream("test","FILE",self.filename)

  def tearDown(self):
    self.logstr.__del__()
    OpenRTM_aist.Manager.instance().shutdownManager()
    pass

  def test_strToLogLevel(self):
    self.logger = Logger.init("test","FILE","test.log")
    self.assertEqual(self.logger.strToLogLevel("SILENT"), Logger.SILENT)
    self.assertEqual(self.logger.strToLogLevel("ERROR"), Logger.ERROR)
    self.assertEqual(self.logger.strToLogLevel("WARN"), Logger.WARN)
    self.assertEqual(self.logger.strToLogLevel("INFO"), Logger.INFO)
    self.assertEqual(self.logger.strToLogLevel("DEBUG"), Logger.DEBUG)
    self.assertEqual(self.logger.strToLogLevel("TRACE"), Logger.TRACE)
    self.assertEqual(self.logger.strToLogLevel("VERBOSE"), Logger.VERBOSE)
    self.assertEqual(self.logger.strToLogLevel("PARANOID"), Logger.PARANOID)
    self.assertEqual(self.logger.strToLogLevel("HOGE"), Logger.INFO)


  def test_addHandler(self):
    #self.logstr.addHandler("stdout")
    self.logstr.setLogLevel("INFO")
    self.logstr.RTC_INFO("addHandler test!!!!!")
    

  def test_setLogLock(self):
    self.logstr.setLogLock(True)
    self.logstr.setLogLock(False)

  def test_enableLogLock(self):
    self.logstr.enableLogLock()

  def test_disableLogLock(self):
    self.logstr.disableLogLock()

  def test_acquire_release(self):
    self.logstr.acquire()
    self.logstr.release()

  def test_RTC_LOG(self):
    import logging
    self.logstr.RTC_LOG(logging.ERROR,"log %s, %s",("hoge","hogehoge"))


  def test_RTC_ERROR(self):
    self.logstr.RTC_ERROR("error!!!!!")
    def test():
      raise MemorryError
    try:
      test()
    except:
      self.logstr.RTC_ERROR(sys.exc_info())


  def test_RTC_WARN(self):
    self.logstr.RTC_WARN("warn!!!!!")


  def test_RTC_INFO(self):
    self.logstr.RTC_INFO("info!!!!!")


  def test_RTC_DEBUG(self):
    self.logstr.RTC_DEBUG("debug!!!!!")


  def test_RTC_TRACE(self):
    self.logstr.RTC_TRACE("trace!!!!")

  def test_RTC_VERBOSE(self):
    self.logstr.RTC_VERBOSE("verbose!!!!")

  def test_RTC_PARANOID(self):
    self.logstr.RTC_PARANOID("paranoid!!!!")


if __name__ == "__main__":
  unittest.main()
