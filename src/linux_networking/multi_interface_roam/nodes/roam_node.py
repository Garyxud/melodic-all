#! /usr/bin/env python

from __future__ import with_statement

import roslib; roslib.load_manifest('multi_interface_roam') 
import rospy
import std_msgs.msg
import multi_interface_roam.multi_interface_roam as mir
import multi_interface_roam.rosmasterless
import sys
import os
import os.path
import time
import yaml
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
from pr2_msgs.msg import AccessPoint

freq_to_chan_map = {
    2412000000L : 1, 
    2417000000L : 2, 
    2422000000L : 3, 
    2427000000L : 4, 
    2432000000L : 5, 
    2437000000L : 6, 
    2442000000L : 7, 
    2447000000L : 8, 
    2452000000L : 9, 
    2457000000L : 10, 
    2462000000L : 11, 
    2467000000L : 12, 
    2472000000L : 13, 
    2484000000L : 14, 
    3657500000L : 131, 
    3662500000L : 132, 
    3660000000L : 132, 
    3667500000L : 133, 
    3665000000L : 133, 
    3672500000L : 134, 
    3670000000L : 134, 
    3677500000L : 135, 
    3682500000L : 136, 
    3680000000L : 136, 
    3687500000L : 137, 
    3685000000L : 137, 
    3689500000L : 138, 
    3690000000L : 138, 
    4915000000L : 183, 
    4920000000L : 184, 
    4925000000L : 185, 
    4935000000L : 187, 
    4940000000L : 188, 
    4945000000L : 189, 
    4960000000L : 192, 
    4980000000L : 196, 
    5035000000L : 7, 
    5040000000L : 8, 
    5045000000L : 9, 
    5055000000L : 11, 
    5060000000L : 12, 
    5080000000L : 16, 
    5170000000L : 34, 
    5180000000L : 36, 
    5190000000L : 38, 
    5200000000L : 40, 
    5210000000L : 42, 
    5220000000L : 44, 
    5230000000L : 46, 
    5240000000L : 48, 
    5260000000L : 52, 
    5280000000L : 56, 
    5300000000L : 60, 
    5320000000L : 64, 
    5500000000L : 100, 
    5520000000L : 104, 
    5540000000L : 108, 
    5560000000L : 112, 
    5580000000L : 116, 
    5600000000L : 120, 
    5620000000L : 124, 
    5640000000L : 128, 
    5660000000L : 132, 
    5680000000L : 136, 
    5700000000L : 140, 
    5745000000L : 149, 
    5765000000L : 153, 
    5785000000L : 157, 
    5805000000L : 161, 
    5825000000L : 165, 
}

class MultiInterfaceDiagPublisher:
    def __init__(self, config_file):
        with open(config_file, 'r') as f:
           self.config = yaml.load(f.read())
        if 'wireless_namespace' in self.config:
            self.wireless_namespace = self.config['wireless_namespace']
        else:
            self.wireless_namespace = 'wifi'

        try:
            mir.logdir = self.config['log_directory']
        except KeyError:
            pass

#        console_output_file = os.path.join(mir.logdir, "console_output.log") 
#        try:
#            if self.config['redirect_console']:
#                print "Redirecting output to:", console_output_file
#                sys.stdout = open(console_output_file, "a", 1)
#                sys.stderr = sys.stdout
#        except KeyError:
#            pass
#        except IOError:
#            print "Error redirecting output to: ", console_output_file

        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray)
        self.wifi_pub = rospy.Publisher(self.wireless_namespace+"/accesspoint", AccessPoint)
        wireless_interfaces = []
        interfaces = self.config['interfaces'] 
        for i in interfaces:
            if interfaces[i]['type'] == 'wireless':
                wireless_interfaces.append(i)
        self.wifi_sub_pub = dict((iface, rospy.Publisher(self.wireless_namespace+"/"+iface+"/accesspoint", AccessPoint)) for iface in wireless_interfaces)

        self.hostname = os.uname()[1]
        print
        print "**************************************************************"
        print mir.log_time_string(time.time()), "restarting."
        print "**************************************************************"
        mir.main(config_file, mir.SimpleSelectionStrategy, self.publish_diags)
        print "**************************************************************"
        print "Exiting."
        print "**************************************************************"
        print
        rospy.signal_shutdown("main has exited")

    @staticmethod
    def fill_diags(name, summary, hid, diags):
        ds = DiagnosticStatus()
        ds.values = [KeyValue(k, str(v)) for (k, v) in diags]
        ds.hardware_id = hid
        ds.name = rospy.get_caller_id().lstrip('/') + ": " + name
        ds.message = summary
        return ds
    
    @staticmethod
    def frequency_to_channel(freq):
        # A bit horrible, but will do until the message changes.
        try:
            return freq_to_chan_map[freq]
        except:
            return -1

    @staticmethod
    def gen_accesspoint_msg(iface):
        msg = AccessPoint()
        msg.essid = iface.essid
        msg.macaddr = iface.bssid
        msg.signal = iface.wifi_signal 
        msg.noise = iface.wifi_noise
        msg.snr = msg.signal - msg.noise
        msg.quality = iface.wifi_quality
        msg.rate = iface.wifi_rate
        msg.tx_power = iface.wifi_txpower
        msg.channel = MultiInterfaceDiagPublisher.frequency_to_channel(iface.wifi_frequency)
        return msg
    
    def publish_diags(self, strategy):
        now = rospy.get_rostime()
        ns = strategy.ns
        ds = self.fill_diags("synthetic interface", ns.diag_summary, self.hostname, strategy.ns.diags)
        ds.level = ns.diag_level
        statuses = [ds]

        for i in range(0, len(ns.interfaces)):
            iface = ns.interfaces[i]
            ds = self.fill_diags(iface.iface, iface.diag_summary, self.hostname, iface.diags)
            statuses.append(ds)
            if iface.__class__ == mir.WirelessInterface:
                msg = self.gen_accesspoint_msg(iface)
                msg.header.stamp = now
                self.wifi_sub_pub[iface.iface].publish(msg)
                if i == ns.active_iface:
                    self.wifi_pub.publish(msg)

        da = DiagnosticArray()
        da.header.stamp = rospy.get_rostime()
        da.status = statuses
        self.diag_pub.publish(da)
        
if __name__ == "__main__":
    if len(sys.argv) != 2:
      print >> sys.stderr
      print >> sys.stderr, "usage: roam_node.py config_file.yaml"
      print >> sys.stderr
      print >> sys.stderr, "This node cannot presently be used with roslaunch as it will be confused by the __name and __log parameters."
      sys.exit(1)
    
    rospy.init_node("multi_interface_roam", disable_rosout=True, disable_rostime=True, disable_signals=True)

    MultiInterfaceDiagPublisher(sys.argv[1])
