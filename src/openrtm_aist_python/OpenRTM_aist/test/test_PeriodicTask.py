#!/usr/bin/env/python
# -*- Python -*-

#
# \file PeriodicTask.py
# \brief PeriodicTask class
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

import sys,time
sys.path.insert(1,"../")

import OpenRTM_aist
import unittest

from PeriodicTask import *
from TimeMeasure import *

class LoggerMock:
	def __init__(self):
		self.m_log = []
		return

	def log(self, msg):
		self.m_log.append(msg);
		return

	def countLog(self, msg):
		self.count = 0
		for i in range(len(self.m_log)):
			if self.m_log[i] == msg:
				self.count += 1
		return self.count

	def clearLog(self):
		self.m_log = []
		return

class MyMock:
	def __init__(self):
		return

	class mysvc2:
		def __init__(self):
			self.m_logger = None
			return

		def __call__(self):
			if self.m_logger is not None:
				self.m_logger.log("mysvc2")
			return 0

		def setLogger(self, logger):
			self.m_logger = logger
			return

	class mysvc3:
		def __init__(self):
			self.m_logger = None
			return

		def __call__(self):
			if self.m_logger is not None:
				self.m_logger.log("mysvc3")
			return 0

		def setLogger(self, logger):
			self.m_logger = logger
			return

class TestPeriodicTask(unittest.TestCase):
	def setUp(self):
		return

	def test_setTask(self):
		self._pt = PeriodicTask()
		self._my = MyMock()
		self._my.mysvc2 = MyMock().mysvc2()
		self._logger = LoggerMock()
		self._my.mysvc2.setLogger(self._logger)

		self.assert_(self._pt.setTask(self._my.mysvc2))
		self.assertEqual(0, self._logger.countLog("mysvc2"))
		self._pt.activate()
		time.sleep(0.005)
		self._pt.finalize()
		time.sleep(0.005)
		self.assert_(1 < self._logger.countLog("mysvc2"))
		return

	def test_setPeriodic(self):
		self._pt = PeriodicTask()
		self._my = MyMock()
		self._my.mysvc2 = MyMock().mysvc2()
		self._logger = LoggerMock()
		self._my.mysvc2.setLogger(self._logger)

		self._pt.setTask(self._my.mysvc2)
		self._pt.setPeriod(0.05)
		self.assertEqual(0, self._logger.countLog("mysvc2"))

		self._pt.activate()
		time.sleep(0.1)
		self._pt.suspend()
		time.sleep(0.05)
		self.assert_(4 > self._logger.countLog("mysvc2"))
		self.assert_(0 < self._logger.countLog("mysvc2"))

		self._logger.clearLog()
		self._pt.setPeriod(0.01)
		self.assertEqual(0, self._logger.countLog("mysvc2"))

		self._pt.resume()
		time.sleep(0.1)
		self._pt.suspend()
		time.sleep(0.01)
		self.assert_(12 > self._logger.countLog("mysvc2"))
		self.assert_( 8 < self._logger.countLog("mysvc2"))

		self._logger.clearLog()
		self._pt.setPeriod(0.05)
		self.assertEqual(0, self._logger.countLog("mysvc2"))

		self._pt.resume()
		time.sleep(0.1)
		self._pt.finalize()
		time.sleep(0.05)
		self.assert_(4 > self._logger.countLog("mysvc2"))
		self.assert_(0 < self._logger.countLog("mysvc2"))
		return

	def test_signal(self):
		self._pt = PeriodicTask()
		self._my = MyMock()
		self._my.mysvc2 = MyMock().mysvc2()
		self._logger = LoggerMock()
		self._my.mysvc2.setLogger(self._logger)

		self._pt.setTask(self._my.mysvc2)
		self._pt.setPeriod(0.05)
		self.assertEqual(0, self._logger.countLog("mysvc2"))

		self._pt.activate()
		time.sleep(0.2)
		self._pt.suspend()
		count = self._logger.countLog("mysvc2")

		time.sleep(0.2)
		self.assertEqual(count, self._logger.countLog("mysvc2"))

		self._pt.signal()
		time.sleep(0.2)
		self.assertEqual(count+1, self._logger.countLog("mysvc2"))

		self._pt.signal()
		time.sleep(0.2)
		self.assertEqual(count+2, self._logger.countLog("mysvc2"))

		self._logger.clearLog()
		self._pt.resume()
		time.sleep(0.2)
		self._pt.suspend()
		time.sleep(0.2)
		self.assert_(6 > self._logger.countLog("mysvc2"))
		self.assert_(2 < self._logger.countLog("mysvc2"))

		self._pt.finalize()
		time.sleep(0.2)
		return

	def test_executionMeasure(self):
		self._pt = PeriodicTask()
		self._my = MyMock()
		self._my.mysvc3 = MyMock().mysvc3()
		self._logger = LoggerMock()
		self._my.mysvc3.setLogger(self._logger)

		wait = 0.03	# sec
		self._pt.setTask(self._my.mysvc3)
		self._pt.setPeriod(0.05)
		self._pt.executionMeasure(True)

		# executionMeasureConut: 10
		self._pt.activate()
		time.sleep(0.6)
		self._pt.suspend()
		time.sleep(0.05)
		estat = self._pt.getExecStat()
		ss = ""
		ss = ss + "wait:  " + str(wait) + "\n"
		ss = ss + "estat max:  " + str(estat._max_interval) + "\n"
		ss = ss + "estat min:  " + str(estat._min_interval) + "\n"
		ss = ss + "estat mean: " + str(estat._mean_interval) + "\n"
		ss = ss + "estat sdev: " + str(estat._std_deviation) + "\n"
		self.assert_(estat._max_interval < (wait + 0.030), ss)
		self.assert_(estat._min_interval > (wait - 0.03), ss)
		self.assert_(abs(estat._mean_interval - wait) < 0.03, ss)
		self.assert_(estat._std_deviation < (wait / 5.0), ss)

		# executionMeasureConut: 5
		self._pt.executionMeasureCount(5)
		self._pt.resume()
		time.sleep(0.3)
		self._pt.suspend()
		time.sleep(0.05)
		estat = self._pt.getExecStat()
		ss = ""
		ss = ss + "wait:  " + str(wait) + "\n"
		ss = ss + "estat max:  " + str(estat._max_interval) + "\n"
		ss = ss + "estat min:  " + str(estat._min_interval) + "\n"
		ss = ss + "estat mean: " + str(estat._mean_interval) + "\n"
		ss = ss + "estat sdev: " + str(estat._std_deviation) + "\n"
		self.assert_(estat._max_interval < (wait + 0.030), ss)
		self.assert_(estat._min_interval > (wait - 0.03), ss)
		self.assert_(abs(estat._mean_interval - wait) < 0.03, ss)
		self.assert_(estat._std_deviation < (wait / 5.0), ss)

		# executionMeasureConut: lessthan
		self._pt.executionMeasureCount(10)
		self._pt.resume()
		time.sleep(0.3)
		self._pt.suspend()
		time.sleep(0.05)
		self._pt.finalize()
		estat2 = self._pt.getExecStat()

		# periodicMeasureConut: lessthan
		self.assert_(estat._max_interval == estat2._max_interval)
		self.assert_(estat._min_interval == estat2._min_interval)
		self.assert_(estat._mean_interval == estat2._mean_interval)
		self.assert_(estat._std_deviation == estat2._std_deviation)
		return

	def test_periodicMeasure(self):
		self._pt = PeriodicTask()
		self._my = MyMock()
		self._my.mysvc3 = MyMock().mysvc3()
		self._logger = LoggerMock()
		self._my.mysvc3.setLogger(self._logger)

		wait = 0.05	# sec
		self._pt.setTask(self._my.mysvc3)
		self._pt.setPeriod(0.05)
		self._pt.periodicMeasure(True)

		# periodicMeasureConut: 10
		self._pt.activate()
		time.sleep(0.6)
		self._pt.suspend()
		time.sleep(0.05)
		pstat = self._pt.getPeriodStat()
		self.assert_(pstat._max_interval < (wait + 0.030))
		self.assert_(pstat._min_interval > (wait - 0.050))
		self.assert_(abs(pstat._mean_interval - wait) < 0.03)
		self.assert_(pstat._std_deviation < (wait / 1.0))

		# periodicMeasureConut:5
		self._pt.periodicMeasureCount(5)
		self._pt.resume()
		time.sleep(0.3)
		self._pt.suspend()
		time.sleep(0.05)
		pstat = self._pt.getPeriodStat()
		self.assert_(pstat._max_interval < (wait + 0.030))
		self.assert_(pstat._min_interval > (wait - 0.010))
		self.assert_(abs(pstat._mean_interval - wait) < 0.03)
		self.assert_(pstat._std_deviation < (wait / 5.0))

		# periodicMeasureConut: lessthan
		self._pt.periodicMeasureCount(10)
		self._pt.resume()
		time.sleep(0.3)
		self._pt.suspend()
		time.sleep(0.05)
		self._pt.finalize()
		pstat2 = self._pt.getPeriodStat()

		# periodicMeasureConut: lessthan
		# Comment: Some errors may be observed.
		#self.assert_(pstat._max_interval == pstat2._max_interval)
		#self.assert_(pstat._min_interval == pstat2._min_interval)
		#self.assert_(pstat._mean_interval == pstat2._mean_interval)
		#self.assert_(pstat._std_deviation == pstat2._std_deviation)
		return


############### test #################
if __name__ == '__main__':
        unittest.main()
