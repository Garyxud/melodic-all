#! /usr/bin/env python

import time

import roslib; roslib.load_manifest('network_control_tests')
import rospy

from network_monitor_udp.linktest import UdpmonsourceHandle
from network_monitor_udp.linktest import LinkTest
from network_monitor_udp.msg import LinktestGoal
from network_traffic_control.projected_link_metrics import get_projected_link_metrics
import dynamic_reconfigure.client

def measure_link(tx_bandwidth, bandwidth_limit, latency, loss,
                 packet_size, max_allowed_latency, max_return_time,
                 sink_ip, direction, duration):
    (proj_bandwidth, proj_latency, proj_loss) = get_projected_link_metrics(bandwidth_limit, latency, loss, \
                                                                           packet_size, tx_bandwidth)
    if proj_latency > max_allowed_latency:
        rospy.logerr("Max allowed latency %.2fms is smaller than projected latency %.2fms,"
                     "results will be flawed, exiting", max_allowed_latency * 1e3, proj_latency * 1e3)
        exit(1)

    dynclient = dynamic_reconfigure.client.Client("tc")
    config = dynclient.update_configuration({"bandwidth_egress" : 0.0, "bandwidth_ingress" : 0.0,
                                             "latency_egress" : 0.0, "latency_ingress" : 0.0,
                                             "loss_egress" : 0.0, "loss_ingress" : 0.0 })
    if config['status'] != "OK":
        rospy.logerr("Initalizing tc node failed: " + config['errmsg'])
        exit(1)
    config = dynclient.update_configuration({"bandwidth_" + direction: bandwidth_limit, 
                                             "latency_" + direction: latency,
                                             "loss_" + direction: loss })
    if config['status'] != "OK":
        rospy.logerr("Setting tc node config failed: " + config['errmsg'])

    srcnode = UdpmonsourceHandle('performance_test')
    test = srcnode.create_test(bw = tx_bandwidth, pktsize = packet_size, duration = duration,
                               sink_ip = sink_ip, sink_port = 12345,
                               bw_type = LinktestGoal.BW_CONSTANT, max_return_time = max_return_time,
                               latencybins = [ max_allowed_latency/4, max_allowed_latency/2, max_allowed_latency])
    test.start()

    time.sleep(duration + 0.5)
    
    rospy.loginfo("Link measurement completed!")
    rospy.loginfo("Link parameters: bandwidth_limit %.2fkbit/s latency %.2fms loss %.2f%% tx_bandwidth %.2fkbit/s\n"
                  "                 packet_size %dbytes max_allowed_latency %.2fms max_return_time %.2fms\n"
                  "                 direction %s duration %.2fs",
                  bandwidth_limit/1e3, latency*1e3, loss, tx_bandwidth/1e3, packet_size, max_allowed_latency*1e3,
                  max_return_time*1e3, direction, duration)
    rospy.loginfo("\nRESULTS: measured_bandwidth %.2fkbit/s measured_latency %.2fms measured_loss %.2f%%",
                  test.bandwidth.avg()/1e3, test.latency.avg() * 1e3, test.loss.avg())

if __name__ == '__main__':
    rospy.init_node('measure_link_node')
    
    tx_bandwidth = rospy.get_param("~tx_bandwidth")

    args = dict()
    args['bandwidth_limit'] = rospy.get_param("~bandwidth_limit")
    args['latency'] = rospy.get_param("~latency")
    args['loss'] = rospy.get_param("~loss")
    args['packet_size'] = rospy.get_param("~packet_size")
    args['max_allowed_latency'] = rospy.get_param("~max_allowed_latency")
    args['max_return_time'] = rospy.get_param("~max_return_time")
    args['sink_ip'] = rospy.get_param("~sink_ip")
    args['direction'] = rospy.get_param("~direction")
    args['duration'] = rospy.get_param("~duration")

    measure_link(tx_bandwidth, **args)
