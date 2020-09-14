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

class AdaptiveBandwidthTest(unittest.TestCase):
    def __init__(self, *args):
        super(AdaptiveBandwidthTest, self).__init__(*args)
        rospy.init_node('network_monitor_udp_test')
        self.srcnode = UdpmonsourceHandle('performance_test')
        self.tc = TcPortControl(self)
        self.tc.reset()

    def setUp(self):
        self.srcnode.cancel_all_tests()
        self.tc.init()

    def tearDown(self):
        self.tc.reset()

    def test_ramp_up(self):
        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_ADAPTIVE, update_interval = 0.2,
                                        latency_threshold = 0.1)
        self.tc.set_rate_limit(5e6)
        test.start()
        time.sleep(5.5)
        self.assertTrue(test.bandwidth.movavg(5) > 4.3e6 and test.bandwidth.movavg(5) < 5.7e6,
                   "Expected capacity on loopback interface to be ~5Mbit/s, instead it was %.2fMbit/s"%
                   (test.bandwidth.movavg(5)/1e6))

    def test_ramp_down(self):
        test = self.srcnode.create_test(bw = 5.0*10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_ADAPTIVE, update_interval = 0.2,
                                        latency_threshold = 0.1)
        self.tc.set_rate_limit(5e6)
        test.start()
        time.sleep(2.0)
        self.tc.set_rate_limit(1e6)
        time.sleep(3.0)
        self.assertTrue(test.bandwidth.movavg(5) > 0.6e6 and test.bandwidth.movavg(5) < 1.4e6,
                   "Expected capacity on loopback interface to be ~1Mbit/s, instead it was %.2fMbit/s"%
                   (test.bandwidth.movavg(5)/1e6))

    def test_varying_capacity(self):
        test = self.srcnode.create_test(bw = 2*10**6, pktsize = 1500, duration = 15.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_ADAPTIVE, update_interval = 0.2,
                                        latency_threshold = 0.1)
        self.tc.set_rate_limit(10e6)
        test.start()
        time.sleep(4.0)
        self.assertTrue(test.bandwidth.movavg(5) > 9e6 and test.bandwidth.movavg(5) < 11e6,
                        "Expected capacity on loopback interface to be ~10Mbit/s, instead it was %.2fMbit/s"%
                        (test.bandwidth.movavg(5)/1e6))
        self.tc.set_rate_limit(1e6)
        time.sleep(4.0)
        self.assertTrue(test.bandwidth.movavg(5) > 0.6e6 and test.bandwidth.movavg(5) < 1.4e6,
                        "Expected capacity on loopback interface to be ~1Mbit/s, instead it was %.2fMbit/s"%
                        (test.bandwidth.movavg(5)/1e6))
        self.tc.set_rate_limit(5e6)
        time.sleep(6.0)
        self.assertTrue(test.bandwidth.movavg(5) > 3.5e6 and test.bandwidth.movavg(5) < 5.7e6,
                        "Expected capacity on loopback interface to be ~5Mbit/s, instead it was %.2fMbit/s"%
                        (test.bandwidth.movavg(5)/1e6))

    def test_get_capacity(self):
        self.tc.set_rate_limit(5e6)
        capacity = self.srcnode.get_link_capacity(sink_ip = "127.0.0.1", sink_port = 12345, latency_threshold = 0.1)
        self.assertTrue(capacity > 4.3e6 and capacity < 5.7e6,
                        "Expected capacity on loopback interface to be ~5Mbit/s, instead it was %.2fMbit/s"%
                        (capacity/1e6))
        self.tc.set_rate_limit(20e6)
        capacity = self.srcnode.get_link_capacity(sink_ip = "127.0.0.1", sink_port = 12345, latency_threshold = 0.1)
        self.assertTrue(capacity > 15e6 and capacity < 25e6,
                        "Expected capacity on loopback interface to be ~20Mbit/s, instead it was %.2fMbit/s"%
                        (capacity/1e6))
        self.tc.set_rate_limit(0.5e6)
        capacity = self.srcnode.get_link_capacity(sink_ip = "127.0.0.1", sink_port = 12345, latency_threshold = 0.1)
        self.assertTrue(capacity > 0.3e6 and capacity < 0.7e6,
                        "Expected capacity on loopback interface to be ~0.5Mbit/s, instead it was %.2fMbit/s"%
                        (capacity/1e6))
                    
if __name__ == '__main__':
    try:
        rostest.run('network_monitor_udp', 'adaptive_bw_test', AdaptiveBandwidthTest)
    except KeyboardInterrupt, e:
        pass
