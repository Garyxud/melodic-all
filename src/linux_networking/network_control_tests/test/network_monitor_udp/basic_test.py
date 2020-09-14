#! /usr/bin/env python

import sys
import time
import subprocess
import unittest

import roslib; roslib.load_manifest('network_monitor_udp')
import rospy
import rostest 

from network_monitor_udp.linktest import UdpmonsourceHandle
from network_monitor_udp.linktest import LinkTest
from network_monitor_udp.msg import LinktestGoal

class BasicTest(unittest.TestCase):
    def __init__(self, *args):
        super(BasicTest, self).__init__(*args)
        rospy.init_node('network_monitor_udp_test')
        self.srcnode = UdpmonsourceHandle('performance_test')

    def setUp(self):
        self.srcnode.cancel_all_tests()
        
    def test_finite_duration(self):
        scheduled_duration = 3.0 # seconds

        test = self.srcnode.create_test("finite_duration_test", 
                                        sink_ip = "127.0.0.1", sink_port = 12345, 
                                        duration = scheduled_duration)
        start_time = time.time()
        test_duration = 0.0 
        test.start()
        while not test.done and test_duration < scheduled_duration + 2.0:
            time.sleep(0.2)
            test_duration = time.time() - start_time

        self.assertTrue( test.done, "Test did not end within time limit. " + 
                    "Scheduled duration was %.2fsec while elapsed duration so far is %.2fsec"% 
                    (scheduled_duration, test_duration) )
        
        self.assertTrue( time.time() - start_time > scheduled_duration - 0.5, 
                       "Test ended too fast, it was scheduled for %.2fsec and it lasted for only  %.2fsec"%
                       (scheduled_duration, time.time() - start_time) )

    def run_periodic_feedback_test(self, update_interval, test_duration = 5.0):        
        self.last_feedback_time = None
        self.feedback_interval_too_small = False
        self.feedback_interval_too_large = False
        self.feedback_interval_too_small_value = 0.0
        self.feedback_interval_too_large_value = 0.0
        self.feedback_calls = 0
        self.update_interval = update_interval

        test = self.srcnode.create_test(sink_ip = "127.0.0.1", sink_port = 12345,
                                        duration = test_duration, update_interval = update_interval)
        test.set_custom_feedback_handler(self.feedback_period_test_handler) 
        test.start()

        while not test.done:
            time.sleep(1.0)

        self.assertFalse(self.feedback_interval_too_large, 
                         "Interval between two feedback calls was too large: expected (%.2f sec) was (%.2f sec)"%
                         (self.update_interval, self.feedback_interval_too_large_value) )

        self.assertFalse(self.feedback_interval_too_small, 
                        "Interval between two feedback calls was too small: expected (%.2f sec) was (%.2f sec)"%
                         (self.update_interval, self.feedback_interval_too_small_value) )
        
        conservative_expected_feedback_calls = int((test_duration - 1.0)/self.update_interval)
        self.assertTrue(self.feedback_calls >= conservative_expected_feedback_calls,
                        "Expected at least (%d) feedback calls, but got only (%d)"%
                        (conservative_expected_feedback_calls, self.feedback_calls))

    def test_feedback_period(self):        
        self.run_periodic_feedback_test(0.5, 5.0)
        self.run_periodic_feedback_test(0.2, 3.0)
        self.run_periodic_feedback_test(0.05, 3.0)

    def feedback_period_test_handler(self, gh, feedback):
        self.feedback_calls += 1
        curr_time = time.time()
        if self.last_feedback_time is not None:
            if curr_time - self.last_feedback_time > self.update_interval * 1.5:
                self.feedback_interval_too_large = True
                self.feedback_interval_too_large_value = curr_time - self.last_feedback_time
            elif curr_time - self.last_feedback_time < self.update_interval * 0.5:
                self.feedback_interval_too_small = True
                self.feedback_interval_too_small_value = curr_time - self.last_feedback_time
        self.last_feedback_time = curr_time

    def test_stop_test(self):
        test = self.srcnode.create_test("finite_duration_test", 
                                        sink_ip = "127.0.0.1", sink_port = 12345, 
                                        duration = 5.0, bw = 1e6 )
        test.start()
        time.sleep(1.0)
        test.stop()
        time.sleep(1.0)
        
        self.assertTrue(test.done,
                        "Test should have been stopped")
        self.assertTrue(test.overall_bandwidth > 0.6e6,
                        "Expected overall bandwidth to be ~1Mbit/s, instead it was %.2fMbit/s"%
                        (test.overall_bandwidth/1e6))
        

if __name__ == '__main__':
    try:
        rostest.run('network_monitor_udp', 'blahblah', BasicTest)
    except KeyboardInterrupt, e:
        pass
