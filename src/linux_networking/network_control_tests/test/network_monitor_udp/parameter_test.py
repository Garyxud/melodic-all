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

import scapy.all as scapy
from tc_port_control import TcPortControl

class ParameterTest(unittest.TestCase):
    def __init__(self, *args):
        super(ParameterTest, self).__init__(*args)
        rospy.init_node('network_monitor_udp_test')
        self.srcnode = UdpmonsourceHandle('performance_test')
        self.tc = TcPortControl(self)
        self.tc.reset()

    def setUp(self):
        self.srcnode.cancel_all_tests()
        self.tc.init()

    def tearDown(self):
        self.tc.reset()

    def test_tos(self):
        time.sleep(1.0)

        test1 = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 1500, duration = 1.0,
                                         sink_ip = "127.0.0.1", sink_port = 12345,
                                         bw_type = LinktestGoal.BW_CONSTANT, 
                                         update_interval = 0.2, tos = 0x10)
        test2 = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 1500, duration = 1.0,
                                         sink_ip = "127.0.0.1", sink_port = 12346,
                                         bw_type = LinktestGoal.BW_CONSTANT, 
                                         update_interval = 0.2, tos = 0x11)
        
        test1.start()
        test2.start()

        s = scapy.L2Socket(iface='lo', filter='udp and port 12345')

        while True:
            pkt = s.recv(2048)
            if pkt is None:
                continue
            if pkt.type != 0x800:
                print "Unexpected protocol 0x%04x"%pkt.type
                continue
            if pkt.dport == 12345:
                tos = pkt.payload.tos
                break

        self.assertEqual(tos, 0x10,
                         "Unexpected TOS value for test1" + str(pkt))

        s = scapy.L2Socket(iface='lo', filter='udp and port 12346')

        while True:
            pkt = s.recv(2048)
            if pkt is None:
                continue
            if pkt.type != 0x800:
                print "Unexpected protocol 0x%04x"%pkt.type
                continue
            if pkt.dport == 12346:
                tos = pkt.payload.tos
                break

        self.assertEqual(tos, 0x11,
                         "Unexpected TOS value for test2: " + str(pkt))

    def test_latencybins(self):
        self.tc.set_latency_loss(latency = 0.15)
        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 500, duration = 1.5,
                                         sink_ip = "127.0.0.1", sink_port = 12345,
                                         bw_type = LinktestGoal.BW_CONSTANT, 
                                         update_interval = 0.2)
        test.start()
        time.sleep(2.0)
        self.assertTrue(test.done, "Expected test to have ended")
        self.assertTrue(test.overall_loss > 90.0,
                        "Expected 100%% loss, instead it was %.2f%%"%(test.overall_loss))
        self.assertTrue(test.latency_histogram[-1] > 0.95 and sum(test.latency_histogram[:-1]) < 0.05,
                        "Expected all packets to be in the last bin (100ms, infinity)")
                    
        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 500, duration = 1.5,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, 
                                        update_interval = 0.2, latencybins = [ 0.01, 0.1, 0.5 ])
        test.start()
        time.sleep(2.0)
        self.assertTrue(test.done, "Expected test to have ended")
        self.assertTrue( test.overall_loss < 2.0,
                         "Expected 0%% loss, instead it was %.2f%%"%
                         (test.overall_loss))

    def test_max_return_time(self):
        self.tc.set_latency_loss(latency = 0.075)
        self.tc.set_latency_loss_udp_returnpath(latency = 0.175)

        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 500, duration = 1.0,
                                         sink_ip = "127.0.0.1", sink_port = 12345,
                                         bw_type = LinktestGoal.BW_CONSTANT, 
                                         update_interval = 0.05, ros_returnpath = False) 
        test.start()
        time.sleep(1.5)
        self.assertTrue(test.done, "Expected test to have ended")
        self.assertEqual(test.latency.min(), 0.0,
                        "Expected min latency to be 0.0ms meaning that for at least one interval "
                        "(in fact, this should hold for all intervals) no packets were restricted [1]")
                   
        
        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 500, duration = 1.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, 
                                        update_interval = 0.05, max_return_time = 0.01)
        test.start()
        time.sleep(1.5)
        self.assertTrue(test.done, "Expected test to have ended")
        self.assertEqual(test.latency.min(), 0.0,
                        "Expected min latency to be 0.0ms meaning that for at least one interval "
                        "(in fact, this should hold for all intervals) no packets were restricted [1]")

        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 500, duration = 1.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, 
                                        update_interval = 0.05, max_return_time = 0.225)
        test.start()
        time.sleep(1.5)
        self.assertTrue(test.done, "Expected test to have ended")
        self.assertTrue(test.latency.min() > 0.05,
                        "Expected min latency to be ~75ms (meaning that there was no interval which had "
                        " no restricted packets), instead it was %.2fms"%
                        (test.latency.min()))
        self.assertTrue(test.loss.min() < 2.0,
                        "Expected 0%% loss during all intervals, instead for one interval it was %.2f%%"%
                        (test.loss.min()) )

    def test_udp_return(self):
        self.tc.set_latency_loss_udp_returnpath(loss = 20.0)

        test = self.srcnode.create_test(bw = 2.0*10**6, pktsize = 500, duration = 1.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, 
                                        update_interval = 0.05, ros_returnpath = False) 
        test.start()
        time.sleep(1.5)
        self.assertTrue(test.done, "Expected test to have ended")
        self.assertTrue(test.loss.movavg() > 10.0,
                        "Expected loss to be ~20%% on UDP return path instead it was %.2f%%"%
                        (test.loss.movavg()))

        test = self.srcnode.create_test(bw = 2.0*10**6, pktsize = 500, duration = 1.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, 
                                        update_interval = 0.05, ros_returnpath = True) 
        test.start()
        time.sleep(1.5)
        self.assertTrue(test.done, "Expected test to have ended")
        self.assertTrue(test.loss.movavg() < 2.0,
                        "Expected loss to be ~0%% with ROS return path instead it was %.2f%%"%
                        (test.loss.movavg()))

    def test_roundtrip(self):
        self.tc.set_latency_loss_udp_returnpath(latency = 0.05)
        
        test = self.srcnode.create_test(bw = 2.0*10**6, pktsize = 500, duration = 1.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, 
                                        update_interval = 0.05, ros_returnpath = False,
                                        roundtrip = False) 
        test.start()
        time.sleep(1.5)
        self.assertTrue(test.done, "Expected test to have ended")
        self.assertTrue(test.latency.movavg() < 0.01,
                        "Expected latency to be < 1ms, instead it was %.2fms"%
                        (test.latency.movavg() * 1e3))

        test = self.srcnode.create_test(bw = 2.0*10**6, pktsize = 500, duration = 1.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, 
                                        update_interval = 0.05, ros_returnpath = False,
                                        roundtrip = True) 
        test.start()
        time.sleep(1.5)
        self.assertTrue(test.done, "Expected test to have ended")
        self.assertTrue(test.latency.movavg() > 0.03,
                        "Expected latency to be ~50ms, instead it was %.2fms"%
                        (test.latency.movavg() * 1e3))

if __name__ == '__main__':
    try:
        rostest.run('network_monitor_udp', 'parameter_test', ParameterTest)
    except KeyboardInterrupt, e:
        pass
