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

class ConstantBandwidthTest(unittest.TestCase):
    def __init__(self, *args):
        super(ConstantBandwidthTest, self).__init__(*args)
        rospy.init_node('network_monitor_udp_test')
        self.srcnode = UdpmonsourceHandle('performance_test')
        self.tc = TcPortControl(self)
        self.tc.reset()

    def setUp(self):
        self.srcnode.cancel_all_tests()
        self.tc.init()

    def tearDown(self):
        self.tc.reset()

    def test_basic(self):
        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 1500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT)
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.overall_latency > 0.0 and test.overall_latency < 0.005,
                        "Expected latency on loopback interface to be positive and under 5ms, instead it was %.2fms"%
                        (test.overall_latency * 1000))
        self.assertAlmostEqual(test.overall_loss, 0.0, 2,
                               "Expected packet loss on loopback interface to be zero, instead it was %.2f%%"%
                               (test.overall_loss))
        self.assertTrue(test.overall_bandwidth > 0.9 * 10**6,
                        "Expected useful bandwidth on loopback interface to be at least 0.9Mbit/s, instead it was %.2fMbit/s"%
                        (test.overall_bandwidth/1e6))
                    

    def test_bw_measurement(self):
        test = self.srcnode.create_test(bw = 3.0*10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT)
        self.tc.set_rate_limit(1e6)
        test.start()
        time.sleep(5.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.overall_bandwidth < 1e6 and test.overall_bandwidth > 0.9e6,
                        "Expected useful bandwidth on loopback interface to be at least 0.9Mbit/s, instead it was %.2fMbit/s"%
                        (test.overall_bandwidth/1e6))

    def test_bw_measurement_variable_bw(self):   
        test = self.srcnode.create_test(bw = 5.0*10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, update_interval = 0.15)
        self.tc.set_rate_limit(1e6)
        test.start()
        time.sleep(2.0)
        self.assertTrue(test.bandwidth.movavg(5) < 1.15e6 and test.bandwidth.movavg(5) > 0.85e6,
                        "Expected useful bandwidth on loopback interface to be ~1Mbit/s, instead it was %.2fMbit/s"%
                        (test.bandwidth.movavg(5)/1e6))
        self.tc.set_rate_limit(4e6)
        time.sleep(2.0)
        self.assertTrue(test.bandwidth.movavg(5) < 4.2e6 and test.bandwidth.movavg(5) > 3.8e6,
                        "Expected useful bandwidth on loopback interface to be ~4Mbit/s, instead it was %.2fMbit/s"%
                        (test.bandwidth.movavg(5)/1e6))
        self.tc.set_rate_limit(0.25e6)
        time.sleep(2.0)
        self.assertTrue(test.bandwidth.movavg(5) < 0.35e6 and test.bandwidth.movavg(5) > 0.15e6,
                        "Expected useful bandwidth on loopback interface to be ~0.25Mbit/s, instead it was %.2fMbit/s"%
                        (test.bandwidth.movavg(5)/1e6))

    def test_loss_measurement(self):
        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 200, duration = 12.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, update_interval = 0.15)
        self.tc.set_latency_loss(loss = 5)
        test.start()
        time.sleep(4.0)
        self.assertTrue(test.loss.movavg() < 8.5 and test.loss.movavg() > 1.5,
                        "Expected packet loss on loopback interface to be ~5%%, instead it was %.2f%%"%
                        (test.loss.movavg()))
        self.tc.set_latency_loss(loss = 20)
        time.sleep(4.0)
        self.assertTrue(test.loss.movavg() < 25.0 and test.loss.movavg() > 15.0,
                        "Expected packet loss on loopback interface to be ~20%%, instead it was %.2f%%"%
                        (test.loss.movavg()))
        self.tc.set_latency_loss(loss = 0)
        time.sleep(4.0)
        self.assertTrue(test.loss.movavg() < 1.0,
                        "Expected packet loss on loopback interface to be ~0%%, instead it was %.2f%%"%
                        (test.loss.movavg()))

    def test_latency_measurement(self):
        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 200, duration = 6.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, update_interval = 0.2)
        test.start()
        time.sleep(1.5)
        lo_latency = test.latency.movavg(3)        
        self.tc.set_latency_loss(latency = 0.02)
        time.sleep(1.5)
        added_latency = test.latency.movavg(3) - lo_latency
        self.assertTrue(added_latency > 0.015 and added_latency < 0.025,
                        "Expected added latency on loopback interface to be ~20ms, instead it was %.2fms"%
                        (added_latency * 1000))
        self.tc.set_latency_loss(latency = 0.06)
        time.sleep(1.5)
        added_latency = test.latency.movavg(3) - lo_latency
        self.assertTrue(added_latency > 0.052 and added_latency < 0.068,
                        "Expected added latency on loopback interface to be ~60ms, instead it was %.2fms"%
                        (added_latency * 1000))
        self.tc.set_latency_loss(latency = 0)
        time.sleep(1.5)
        added_latency = test.latency.movavg(3) - lo_latency
        self.assertTrue(added_latency < 0.002,
                        "Expected added latency on loopback interface to be ~0ms, instead it was %.2fms"%
                        (added_latency * 1000))

    def test_latency_loss(self):
        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 200, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, update_interval = 0.2)
        self.tc.set_latency_loss(loss = 5, latency = 0.05)
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.loss.movavg(10) < 7.5 and test.loss.movavg(10) > 2.5,
                        "Expected packet loss on loopback interface to be ~5%%, instead it was %.2f%%"%
                        (test.loss.movavg(10)))
        self.assertTrue(test.latency.movavg(10) < 0.07 and test.latency.movavg(10) > 0.05,
                        "Expected latency on loopback interface to be ~60ms, instead it was %.2fms"%
                        (test.latency.movavg(10)* 1000))

if __name__ == '__main__':
    try:
        rostest.run('network_monitor_udp', 'blahblah', ConstantBandwidthTest)
    except KeyboardInterrupt, e:
        pass
