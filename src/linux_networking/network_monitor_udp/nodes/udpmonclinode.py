#! /usr/bin/env python


import roslib; roslib.load_manifest('network_monitor_udp')
import rospy

import sys
import network_monitor_udp.msg as msgs
from network_monitor_udp.udpmoncli import MonitorClient

if __name__ == "__main__":
    try:
    	rospy.init_node('udpmonclinode', anonymous=True)
        host = rospy.get_param("~host")
        port = int(rospy.get_param("~port", 1234))
        rate = float(rospy.get_param("~pkt_rate", 100.0)) 
        size = int(rospy.get_param("~pkt_size", 32))
        interface = rospy.get_param("~interface", "0.0.0.0")
        update_rate = float(rospy.get_param("~update_rate", 2.0))
        bin_limits = rospy.get_param("~bin_limits", "0.005 0.01 0.1 0.2 0.3 0.4 0.5")
        bin_limits = [float(val) for val in bin_limits.split()]
        cli = MonitorClient(bin_limits, (host, port), rate, size, sourceaddr = (interface, 0))
        pub = rospy.Publisher('udpmonitor', msgs.UdpMonitor)
        try:
            display_interval_sec = 1.0 / update_rate
            display_interval = rospy.Duration(display_interval_sec)
            start_time = rospy.Time.now()
            next_time = start_time
            msg = msgs.UdpMonitor()
            msg.server_host = host
            msg.server_port = port
            msg.source_interface = interface
            msg.latency_bin_limits = bin_limits
            msg.packet_size = size
            msg.packet_rate = rate
            
            while not rospy.is_shutdown():
                next_time = next_time + display_interval
                sleeptime = next_time - rospy.Time.now()
                rospy.sleep(sleeptime)
                msg.latency_bin_values, msg.average_latency_all, msg.average_latency_fresh = \
                        cli.get_smart_bins(display_interval_sec)
                msg.header.stamp = rospy.Time.now()
                msg.loss_fresh = 1. - sum(msg.latency_bin_values[:-1])
                pub.publish(msg)
        
        finally:
            cli.shutdown()

    except KeyboardInterrupt:
        print >> sys.stderr, "Exiting on CTRL+C."

