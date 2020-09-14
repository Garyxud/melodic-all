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

from tc_port_control import TcPortControl

class MultipleEndsTest(unittest.TestCase):
    def __init__(self, *args):
        super(MultipleEndsTest, self).__init__(*args)
        rospy.init_node('network_monitor_udp_test')
        self.srcnode = UdpmonsourceHandle('performance_test')
        self.tc = TcPortControl(self)
        self.tc.reset()

    def setUp(self):
        self.srcnode.cancel_all_tests()
        self.tc.init()

    def tearDown(self):
        self.tc.reset()

    def test_multiple_sources_single_sink(self):
        self.tc.set_rate_limit(1e6)

        test1 = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 1500, duration = 5.0,
                                         sink_ip = "127.0.0.1", sink_port = 12345,
                                         bw_type = LinktestGoal.BW_CONSTANT, 
                                         update_interval = 0.2)
        test2 = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 1500, duration = 10.0,
                                         sink_ip = "127.0.0.1", sink_port = 12345,
                                         bw_type = LinktestGoal.BW_CONSTANT, 
                                         update_interval = 0.2)
        test3 = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 1500, duration = 15.0,
                                         sink_ip = "127.0.0.1", sink_port = 12345,
                                         bw_type = LinktestGoal.BW_CONSTANT, 
                                         update_interval = 0.2)
        test1.start()
        test2.start()
        test3.start()

        time.sleep(3.0)

        bwsum = test1.bandwidth.movavg(5) + test2.bandwidth.movavg(5) + test3.bandwidth.movavg(5)
        self.assertTrue(bwsum > 0.8e6 and bwsum < 1.2e6,
                        "Expected the combined bandwidth of three tests to be ~1Mbit/s, instead it was %.2fMbit/s"%
                        (bwsum))

        time.sleep(2.5)
        
        self.assertTrue(test1.done,
                        "Expected test1 to have finished at time = 5.5s")
        self.assertFalse(test2.done,
                         "Expected test2 to be still running at time = 5.5s")
        self.assertFalse(test3.done,
                         "Expected test3 to be still running at time = 5.5s")

        time.sleep(2.5)
        
        bwsum = test2.bandwidth.movavg(5) + test3.bandwidth.movavg(5)
        self.assertTrue(bwsum > 0.8e6 and bwsum < 1.2e6,
                        "Expected the combined bandwidth of two tests to be ~1Mbit/s, instead it was %.2fMbit/s"%
                        (bwsum))

        time.sleep(2.5)

        self.assertTrue(test2.done,
                        "Expected test2 to have finished at time = 10.5s")
        self.assertFalse(test3.done,
                         "Expected test3 to be still running at time = 10.5s")

        time.sleep(2.5)

        self.assertTrue(test3.bandwidth.movavg(5) > 0.80e6 and test3.bandwidth.movavg(5) < 1.20e6,
                        "Expected test3 bandwidth to be ~1Mbit/s (with 1 test running concurrently)" + 
                        ", instead it was %.2fMbit/s"%
                        (test3.bandwidth.movavg(5)/1e6))

        time.sleep(2.5)

        self.assertTrue(test2.done,
                        "Expected test2 to have finished at time = 10.5s")

        self.assertTrue(test3.latency.duration() > test2.latency.duration() and 
                        test2.latency.duration() > test1.latency.duration(),
                        "Expected test3 duration (%.2fs) to be greater than test2 (%.2fs)"
                        " and still greater than test1 (%.2fs)"%
                        (test3.latency.duration(), test2.latency.duration(), test1.latency.duration()))
        
    def test_multiple_sinks(self):
        self.tc.set_rate_limit(1e6)

        test1 = self.srcnode.create_test(bw = 5.0*10**6, pktsize = 1500, duration = 3.0,
                                         sink_ip = "127.0.0.1", sink_port = 12345,
                                         bw_type = LinktestGoal.BW_CONSTANT, 
                                         update_interval = 0.2)
        test2 = self.srcnode.create_test(bw = 5.0*10**6, pktsize = 1500, duration = 3.0,
                                         sink_ip = "127.0.0.1", sink_port = 12346,
                                         bw_type = LinktestGoal.BW_CONSTANT, 
                                         update_interval = 0.2)
        test1.start()
        test2.start()
        time.sleep(3.5)
        
        self.assertTrue(test1.done, "Expected test2 to have finished at time = 3.5s")
        self.assertTrue(test2.done, "Expected test2 to have finished at time = 3.5s")
        
        self.assertTrue(test2.overall_bandwidth > 4 * test1.overall_bandwidth,
                        "Expected test2 bw (%.2fMbit/s) to be at least 4x larger"
                        " than test1 bw (%.2fMbit/s)"%
                        (test2.overall_bandwidth/1e6, test1.overall_bandwidth/1e6))

if __name__ == '__main__':
    try:
        rostest.run('network_monitor_udp', 'multiple_ends_test', MultipleEndsTest)
    except KeyboardInterrupt, e:
        pass
