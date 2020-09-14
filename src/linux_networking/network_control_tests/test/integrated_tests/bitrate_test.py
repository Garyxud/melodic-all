#! /usr/bin/env python

import sys
import time
import subprocess

import roslib; roslib.load_manifest('network_monitor_udp')
import rospy
from network_monitor_udp.linktest import UdpmonsourceHandle
from network_monitor_udp.linktest import LinkTest
from network_monitor_udp.msg import LinktestGoal
import dynamic_reconfigure.client

AP_PREFIX="/ap_atheros/"
HOSTAPD_NODE=AP_PREFIX+"ap_control"
AP_SINK_IP="192.168.69.1"
AP_SINK_PORT=12345

STA_PREFIX="/sta/"
STA_IFACE="eth1"
STA_SINK_IP="192.168.69.2"
STA_SINK_PORT=12345

bitrates = [ int(rate*10**6) for rate in [ 1, 2, 5.5, 6, 9, 11, 12, 18, 24, 36, 48, 54 ] ]

DEFAULT_PACKET_SIZE = 1500

def run_link_capacity_test(direction = "down", rate = bitrates[0]):
    if direction == "up":
        ret = subprocess.call(["iwconfig", STA_IFACE, "rate", str(rate)])
        if ret != 0:
            raise IOError("iwconfig could not set bitrate on " + STA_IFACE)
        return sta_source.get_link_capacity(sink_ip=AP_SINK_IP, sink_port=AP_SINK_PORT, rostopic_prefix=AP_PREFIX, pktsize=pktsize)
    elif direction == "down":
        config = dynclient.update_configuration({"bitrate": rate})
        if config["status"] != "OK":
            raise ValueError(config["errmsg"])
        return ap_source.get_link_capacity(sink_ip=STA_SINK_IP, sink_port=STA_SINK_PORT, rostopic_prefix=STA_PREFIX, pktsize=pktsize)
        
def setup_hostapd_ap():        
    config = dynclient.update_configuration({"enabled": True, "ssid": "testnet", "mode": "g"})
    if config["status"] != "OK":
        raise ValueError(config["errmsg"])

if __name__ == '__main__':
    pktsize = DEFAULT_PACKET_SIZE
    if len(sys.argv) == 2:
        pktsize = int(sys.argv[1])

    rospy.init_node('testnode')

    dynclient = dynamic_reconfigure.client.Client(HOSTAPD_NODE)
    
    ap_source = UdpmonsourceHandle('/ap_atheros/performance_test') 
    sta_source = UdpmonsourceHandle('/sta/performance_test') 
    ap_source.cancel_all_tests()
    sta_source.cancel_all_tests()

    setup_hostapd_ap()

    print "# Link capacity test"
    print "# Packet size: %d bytes"%(pktsize)
    print "# bitrate uplink_capacity downlink_capacity"
    for rate in bitrates:
        print rate, run_link_capacity_test("up", rate), run_link_capacity_test("down", rate)

