#! /usr/bin/env python

import roslib; roslib.load_manifest('network_monitor_udp')
import rospy

from network_monitor_udp.linktest import UdpmonsourceHandle
from network_monitor_udp.linktest import LinkTest
from network_monitor_udp.msg import LinktestGoal

if __name__ == '__main__':
    rospy.init_node('test_node')
        
    source = UdpmonsourceHandle() 
    source.cancel_all_tests()
    
    try:
        print "Link capacity: %.2fMbit/s"%(source.get_link_capacity(sink_ip="127.0.0.1", sink_port=12345)/1e6)
    finally:
        source.cancel_all_tests()
