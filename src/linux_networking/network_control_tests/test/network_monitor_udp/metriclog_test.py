#! /usr/bin/env python

import sys
import time
import subprocess
import unittest
import math

import roslib; roslib.load_manifest('network_monitor_udp')
import rospy
import rostest 

from network_monitor_udp.linktest import MetricLog
from network_monitor_udp.linktest import UdpmonsourceHandle
from network_monitor_udp.linktest import LinkTest
from network_monitor_udp.msg import LinktestGoal

class MetricLogTest(unittest.TestCase):
    def __init__(self, *args):
        super(MetricLogTest, self).__init__(*args)

    def setUp(self):
        pass
        
    def test_basic_stats(self):
        testlog = MetricLog()

        for i in range(1, 101):
            testlog.record(float(i), rospy.Time(float(i)))

        self.assertEqual(testlog.curr(), 100.0,
                         "Curr val should be (%.2f) and was instead (%.2f)"%
                         (100.0, testlog.curr()))

        self.assertEqual(testlog.min(), 1.0, 
                         "Min val should be (%.2f) and was instead (%.2f)"%
                         (1.0, testlog.min()))

        self.assertEqual(testlog.max(), 100.0, 
                         "Max val should be (%.2f) and was instead (%.2f)"%
                         (100.0, testlog.max()))

        self.assertEqual(testlog.avg(), 50.5, 
                         "Avg val should be (%.2f) and was instead (%.2f)"%
                         (50.5, testlog.avg()))

        self.assertEqual(testlog.count(), 100, 
                         "Count should be (%d) and was instead (%d)"%
                         (100, testlog.count()))

        self.assertAlmostEqual(testlog.stdev(), 29.0115, 3, 
                               "Stdev should be (%.2f) and was instead (%.2f)"%
                               (29.0115, testlog.stdev()))

        for i in range(101, 201):
            testlog.record(-float(i), rospy.Time(float(i)))

        self.assertEqual(testlog.curr(), -200.0,
                         "Curr val should be (%.2f) and was instead (%.2f)"%
                         (-200.0, testlog.curr()))

        self.assertEqual(testlog.min(), -200.0, 
                         "Min val should be (%.2f) and was instead (%.2f)"%
                         (1.0, testlog.min()))

        self.assertEqual(testlog.max(), 100.0, 
                         "Max val should be (%.2f) and was instead (%.2f)"%
                         (100.0, testlog.max()))

        self.assertEqual(testlog.avg(), -50.0, 
                         "Avg val should be (%.2f) and was instead (%.2f)"%
                         (-50.0, testlog.avg()))

        self.assertEqual(testlog.count(), 200, 
                         "Count should be (%d) and was instead (%d)"%
                         (200, testlog.count()))

        self.assertAlmostEqual(testlog.stdev(), 104.8258, 3, 
                               "Stdev should be (%.2f) and was instead (%.2f)"%
                               (104.8258, testlog.stdev()))

    def test_moving_stats(self):
        testlog = MetricLog()
        
        for i in range(1, 98, 4):
            testlog.record(float(i), rospy.Time(float(i)))
            
        self.assertEqual(testlog.movavg(), 79.0,
                         "Moving avg should be (%.2f) and was instead (%.2f)"%
                         (79.0, testlog.movavg()))

        self.assertAlmostEqual(testlog.movstdev(), 12.1106, 3, 
                               "Stdev should be (%.2f) and was instead (%.2f)"%
                               (12.1106, testlog.movstdev()))

        self.assertAlmostEqual(testlog.expavg(), 63.8716, 3, 
                               "Expavg should be (%.2f) and was instead (%.2f)"%
                               (63.8716, testlog.expavg()))

    def test_time_and_duration(self):
        testlog = MetricLog()

        testlog.min_time()

        self.assertEqual(testlog.min_time(), 0.0)
        self.assertEqual(testlog.max_time(), 0.0)
        self.assertEqual(testlog.duration(), 0.0,
                         "Duration should be (%.2f) and was instead (%.2f)"%
                         (0.0, testlog.duration()))

        testlog.record(1.0, 5.0)
        testlog.record(2.0, 1000.0)
        testlog.record(3.0, 1330.0)

        self.assertEqual(testlog.min_time(), 5.0,
                         "Min recorded time should be (%.2f) and was instead (%.2f)"%
                         (5.0, testlog.min_time()))
        self.assertEqual(testlog.max_time(), 1330.0,
                         "Min recorded time should be (%.2f) and was instead (%.2f)"%
                         (1330.0, testlog.max_time()))
        self.assertEqual(testlog.duration(), 1325.0,
                         "Duration should be (%.2f) and was instead (%.2f)"%
                         (1325.0, testlog.duration()))

        testlog.reset_all()

        start_time = time.time()
        testlog.record(1.0)
        time.sleep(0.5)
        testlog.record(2.0)
        time.sleep(0.5)
        testlog.record(3.0)
        end_time = time.time()

        self.assertTrue(math.fabs(end_time - testlog.max_time()) < 0.1,
                        "Max time should be (%.2f) and was instead (%.2f)"%
                        (end_time, testlog.max_time()))

        self.assertTrue(math.fabs(end_time - testlog.max_time()) < 0.1,
                        "Min time should be (%.2f) and was instead (%.2f)"%
                        (end_time, testlog.min_time()))
 
        self.assertTrue(math.fabs((end_time - start_time) - testlog.duration()) < 0.15, 
                        "Duration should be (%.2f) and was instead (%.2f)"%
                        ((end_time - start_time), testlog.duration()))
                       
if __name__ == '__main__':
    try:
        rostest.run('network_monitor_udp', 'blahblah', MetricLogTest)
    except KeyboardInterrupt, e:
        pass
