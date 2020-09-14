#! /usr/bin/env python

import sys
import time
import subprocess
import unittest

import roslib; roslib.load_manifest('network_control_tests')
import rospy
import rostest 
import math

from network_monitor_udp.linktest import UdpmonsourceHandle
from network_monitor_udp.linktest import LinkTest
from network_monitor_udp.msg import LinktestGoal
from network_traffic_control.projected_link_metrics import get_projected_link_metrics
import dynamic_reconfigure.client

class DynreconfTest(unittest.TestCase):
    def __init__(self, *args):
        super(DynreconfTest, self).__init__(*args)
        rospy.init_node('network_traffic_control_test')
        self.srcnode = UdpmonsourceHandle('performance_test')
        self.dynclient = dynamic_reconfigure.client.Client("tc_lo")
        self.no_traffic_control_params = { 
            "bandwidth_egress" : 0.0, "bandwidth_ingress" : 0.0,
            "latency_egress" : 0.0, "latency_ingress" : 0.0,
            "loss_egress" : 0.0, "loss_ingress" : 0.0,
            "packet_size" : 1500 }
        self.reset_tc_rules_via_dynreconf()
        
    def reset_tc_rules_via_dynreconf(self):        
        config = self.dynclient.update_configuration(self.no_traffic_control_params)
        if config['status'] != "OK":
            raise(ValueError(config['errmsg']))

    def setUp(self):
        self.srcnode.cancel_all_tests()

    def tearDown(self):
        self.reset_tc_rules_via_dynreconf()

    def test_latency(self):
        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 1500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT)
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.overall_latency < 0.01, 
                        "Expected latency to be < 1ms, instead it was %.2fms"%
                        (test.overall_latency * 1e3))

        config = self.dynclient.update_configuration({ "latency_egress": 0.03 })
        self.assertTrue(config['status'] == "OK",
                        "Operation FAILed: " + config['errmsg'])
        self.assertAlmostEqual(config['latency_egress'], 0.03, 3,
                               "Expected latency_egress dynreconf parameter to be ~30ms, instead it was %.2fms"%
                               (config['latency_egress'] * 1e3))

        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 1500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT)
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.overall_latency > 0.02 and test.overall_latency < 0.04, 
                        "Expected latency to be ~30ms, instead it was %.2fms"%
                        (test.overall_latency * 1e3))
        self.assertTrue(test.overall_bandwidth > 0.7e6,
                        "Expected overall bandwidth to be ~1Mbit/s, instead it was %.2fMbit/s"%
                        (test.overall_bandwidth/1e6))
        self.assertTrue(test.overall_loss < 5.0,
                        "Expected packet loss to be 0%%, instead it was %.2f%%"%
                        (test.overall_loss))

        config = self.dynclient.update_configuration({ "latency_egress": 0.01, "latency_ingress": 0.05 })
        self.assertTrue(config['status'] == "OK",
                        "Operation FAILed: " + config['errmsg'])
        self.assertAlmostEqual(config['latency_egress'], 0.01, 3,
                               "Expected latency_egress dynreconf parameter to be ~10ms, instead it was %.2fms"%
                               (config['latency_egress'] * 1e3))        
        self.assertAlmostEqual(config['latency_ingress'], 0.05, 3,
                               "Expected latency_ingress dynreconf parameter to be ~50ms, instead it was %.2fms"%
                               (config['latency_ingress'] * 1e3))        

        test = self.srcnode.create_test(bw = 1.0*10**6, pktsize = 1500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, max_return_time = 0.1)
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.overall_latency > 0.045 and test.overall_latency < 0.075, 
                        "Expected latency to be ~ 60ms, instead it was %.2fms"%
                        (test.overall_latency * 1e3))
        self.assertTrue(test.overall_bandwidth > 0.7e6,
                        "Expected overall bandwidth to be ~1Mbit/s, instead it was %.2fMbit/s"%
                        (test.overall_bandwidth/1e6))
        self.assertTrue(test.overall_loss < 5.0,
                        "Expected packet loss to be 0%%, instead it was %.2f%%"%
                        (test.overall_loss))

    def test_bandwidth(self):
        test = self.srcnode.create_test(bw = 3.0*10**6, pktsize = 1500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT)
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.overall_latency < 0.01, 
                        "Expected latency to be < 1ms, instead it was %.2fms"%
                        (test.overall_latency * 1e3))
        self.assertTrue(test.overall_bandwidth > 2.7e6,
                        "Expected overall bandwidth to be ~3Mbit/s, instead it was %.2fMbit/s"%
                        (test.overall_bandwidth/1e6))

        config = self.dynclient.update_configuration({ "bandwidth_egress": 2e6, "bandwidth_ingress": 2e6 })
        self.assertTrue(config['status'] == "OK",
                        "Operation FAILed: " + config['errmsg'])
        self.assertAlmostEqual(config['bandwidth_egress']/1e6, 2.0, 3,
                               "Expected bandwidth_egress dynreconf parameter to be ~2Mbit/s, instead it was %.2fMbit/s"%
                               (config['bandwidth_egress']/1e6))        
        self.assertAlmostEqual(config['bandwidth_ingress']/1e6, 2.0, 3,
                               "Expected bandwidth_ingress dynreconf parameter to be ~2Mbit/s, instead it was %.2fMbit/s"%
                               (config['bandwidth_ingress']/1e6))        

        test = self.srcnode.create_test(bw = 3.0*10**6, pktsize = 1500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, latencybins = [ 0.01, 0.1, 0.3],
                                        max_return_time = 0.25)
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.overall_bandwidth > 1.5e6 and test.overall_bandwidth < 2.5e6,
                        "Expected measured bandwidth to be ~2Mbit/s, instead it was %.2fMbit/s"%
                        (test.overall_bandwidth/1e6))

        config = self.dynclient.update_configuration({ "bandwidth_ingress": 1e6 })

        test = self.srcnode.create_test(bw = 3.0*10**6, pktsize = 1500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT)
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.overall_bandwidth > 0.7e6 and test.overall_bandwidth < 1.3e6,
                        "Expected overall bandwidth to be ~1Mbit/s, instead it was %.2fMbit/s"%
                        (test.overall_bandwidth/1e6))

    def test_bandwidth_latency(self):
        config = self.dynclient.update_configuration({ "bandwidth_egress": 1e6, "latency_ingress": 0.03 })
        test = self.srcnode.create_test(bw = 0.7*10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, max_return_time = 0.01)
        test.start()
        time.sleep(5.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.overall_bandwidth > 0.5e6 and test.overall_bandwidth < 0.9e6,
                        "Expected overall bandwidth to be ~1Mbit/s, instead it was %.2fMbit/s"%
                        (test.overall_bandwidth/1e6))
        self.assertTrue(test.latency.avg() > 0.025  and test.latency.avg() < 0.045,
                        "Expected latency to be ~30ms (since capacity was not overrun), instead it was %.2fms"%
                        (test.latency.avg()*1e3))
        test = self.srcnode.create_test(bw = 3.0*10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, max_return_time = 0.01)
        test.start()
        time.sleep(5.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.overall_bandwidth > 0.8e6 and test.overall_bandwidth < 1.2e6,
                        "Expected overall bandwidth to be ~1Mbit/s, instead it was %.2fMbit/s"%
                        (test.overall_bandwidth/1e6))
        expected_latency = 1500.0/(1e6/8) + 0.03
        self.assertTrue(test.latency.avg() > expected_latency - 0.005  and test.latency.avg() < expected_latency + 0.005,
                        "Expected latency to be ~%.2fms (since capacity was not overrun), instead it was %.2fms"%
                        (expected_latency * 1e3, test.latency.avg()*1e3))

    def test_ingress_egress_loss(self):
        config = self.dynclient.update_configuration({ "loss_egress": 20.0, "loss_ingress": 30.0 })
        test = self.srcnode.create_test(bw = 5*10**6, pktsize = 1500, duration = 5.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, max_return_time = 0.01)
        test.start()
        time.sleep(5.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.loss.avg() > 44.0 - 10.0 and test.loss.avg() < 44.0 + 10.0,
                        "Expected aggregated loss (ingress+egress) to be ~44%%, instead it was %.2f%%"%
                        (test.loss.avg()))

    def test_packet_size(self):
        test = self.srcnode.create_test(bw = 3*10**6, pktsize = 500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, max_return_time = 0.01)
        config = self.dynclient.update_configuration({ "packet_size": 5000, "bandwidth_egress": 1e6 })
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.latency.avg() > 0.035 and test.latency.avg() < 0.045, 
                        "Expected latency to be ~40ms, instead it was %.2fms"%
                        (test.latency.avg()*1e3))

        test = self.srcnode.create_test(bw = 3*10**6, pktsize = 500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, max_return_time = 0.01)
        config = self.dynclient.update_configuration({ "packet_size": 500 })

        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.latency.avg() > 0.002 and test.latency.avg() < 0.006, 
                        "Expected latency to be ~4ms, instead it was %.2fms"%
                        (test.latency.avg()*1e3))

        test = self.srcnode.create_test(bw = 3*10**6, pktsize = 500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, max_return_time = 0.01)
        config = self.dynclient.update_configuration({ "bandwidth_ingress": 0.5e6 })
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.latency.avg() > 0.008 and test.latency.avg() < 0.015, 
                        "Expected latency to be ~12ms, instead it was %.2fms"%
                        (test.latency.avg()*1e3))

        test = self.srcnode.create_test(bw = 3*10**6, pktsize = 500, duration = 3.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, max_return_time = 0.01)
        config = self.dynclient.update_configuration({ "packet_size": 300 })
        test.start()
        time.sleep(3.5)
        self.assertTrue(test.done, "Test should have finished already")
        self.assertTrue(test.loss.avg() > 95.0, 
                        "Expected loss to be 100% (tc limit smaller than packet size)")

    def bandwidth_latency_loss_test(self, direction, bw_tx, bw_limit, latency, loss,
                                    expected_bw = None, expected_loss = None, expected_latency = None):
        self.reset_tc_rules_via_dynreconf()
        config = self.dynclient.update_configuration({ "bandwidth_" + direction: bw_limit, 
                                                       "latency_" + direction: latency,
                                                       "loss_" + direction: loss })

        self.assertTrue(config['status'] == "OK",
                        "Operation FAILed: " + config['errmsg'])

        test = self.srcnode.create_test(bw = bw_tx, pktsize = 1500, duration = 5.0,
                                        sink_ip = "127.0.0.1", sink_port = 12345,
                                        bw_type = LinktestGoal.BW_CONSTANT, max_return_time = 0.01)
        test.start()
        time.sleep(5.5)
        self.assertTrue(test.done, "Test should have finished already")

        (expected_bw, expected_latency, expected_loss) = get_projected_link_metrics(bw_limit, latency, loss, 1500.0, bw_tx)

        print "bw_limit: ", bw_limit, "loss: ", loss, "latency: ", latency, "bw_tx: ", bw_tx
        print "meas_bw: ", test.bandwidth.avg(), " meas_latency: ", test.latency.avg()*1e3, " meas_loss: ", test.loss.avg()
        print "exp_bw", expected_bw, " exp_latency: ", expected_latency * 1e3, " exp_loss: ", expected_loss
        self.assertTrue(test.bandwidth.avg() > expected_bw * 0.75  and test.bandwidth.avg() < expected_bw * 1.25,
                        "Expected measured bandwidth to be ~%.2fMbit/s, instead it was %.2fMbit/s"%
                        (expected_bw/1e6, test.bandwidth.avg()/1e6))
        self.assertTrue(test.latency.avg() > expected_latency - 0.015  and test.latency.avg() < expected_latency + 0.015,
                        "Expected latency to be ~%.2fms, instead it was %.2fms"%
                        (expected_latency * 1e3, test.latency.avg() * 1e3))
        self.assertTrue((test.loss.avg() < 4.0 and expected_loss < 2.0) or 
                        (test.loss.avg() > expected_loss - 10.0  and test.latency.avg() < expected_loss + 10.0),
                        "Expected loss to be ~%.2f%%, instead it was %.2f%%"%
                        (expected_loss, test.loss.avg()))

        
    def test_bandwidth_latency_loss(self):
        # just bw_limit specified, bw_tx < bw_limit
        self.bandwidth_latency_loss_test("egress", bw_tx = 0.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.0, loss = 0.0)
        # just bw_limit specified, bw_tx > bw_limit
        self.bandwidth_latency_loss_test("egress", bw_tx = 1.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.0, loss = 0.0)
        # bw_limit + latency, bw_tx < bw_limit
        self.bandwidth_latency_loss_test("egress", bw_tx = 0.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.01, loss = 0.0)
        # bw_limit + latency, bw_tx > bw_limit
        self.bandwidth_latency_loss_test("egress", bw_tx = 1.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.01, loss = 0.0)
        # bw_limit + latency + loss, bw_tx < bw_limit, loss < bw_loss
        self.bandwidth_latency_loss_test("egress", bw_tx = 0.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.03, loss = 10.0)
        # bw_limit + latency + loss, bw_tx < bw_limit, loss > bw_loss
        self.bandwidth_latency_loss_test("egress", bw_tx = 0.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.03, loss = 50.0)
        # bw_limit + latency + loss, bw_tx > bw_limit, loss < bw-loss
        self.bandwidth_latency_loss_test("egress", bw_tx = 1.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.03, loss = 10.0)
        # bw_limit + latency + loss, bw_tx > bw_limit, loss < bw-loss
        self.bandwidth_latency_loss_test("egress", bw_tx = 1.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.03, loss = 50.0)
        # ingress tests
        self.bandwidth_latency_loss_test("ingress", bw_tx = 0.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.0, loss = 0.0)
        self.bandwidth_latency_loss_test("ingress", bw_tx = 1.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.0, loss = 0.0)
        self.bandwidth_latency_loss_test("ingress", bw_tx = 0.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.01, loss = 0.0)
        self.bandwidth_latency_loss_test("ingress", bw_tx = 1.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.01, loss = 0.0)
        self.bandwidth_latency_loss_test("ingress", bw_tx = 0.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.02, loss = 0.0)
        self.bandwidth_latency_loss_test("ingress", bw_tx = 1.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.03, loss = 10.0)
        self.bandwidth_latency_loss_test("ingress", bw_tx = 0.5*10**6, 
                                         bw_limit=1.0*10**6, latency = 0.04, loss = 20.0)
if __name__ == '__main__':
    try:
        rostest.run('network_control_tests', 'dynreconf_test', DynreconfTest)
    except KeyboardInterrupt, e:
        pass
