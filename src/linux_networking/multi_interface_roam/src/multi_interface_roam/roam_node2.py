#! /usr/bin/env python

#import roslib; roslib.load_manifest('multi_interface_roam')
import rospy
import config
import dynamic_reconfigure.server
import twisted.internet.reactor as reactor
from multi_interface_roam.cfg import MultiInterfaceRoamConfig
from multi_interface_roam.msg import MultiInterfaceStatus, InterfaceStatus
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray
from pr2_msgs.msg import AccessPoint
import mac_addr
import interface_selector
import sys
from std_msgs.msg import Int32
import sigblock
import signal
import interface
from netlink_monitor import IFSTATE
from ieee80211_channels.channels import IEEE80211_Channels
import os

##### monkey-patch to suppress threading error message in python 2.7.3
##### See http://stackoverflow.com/questions/13193278/understand-python-threading-bug
if sys.version_info[:3] == (2, 7, 3):
    import threading
    threading._DummyThread._Thread__stop = lambda x: 42
#####

# Make sure states are nice and consistent...
assert(InterfaceStatus.STATE_NO_INTERFACE == -1)
assert(IFSTATE.PLUGGED == InterfaceStatus.STATE_PLUGGED)
assert(IFSTATE.UP == InterfaceStatus.STATE_UP)
assert(IFSTATE.LINK == InterfaceStatus.STATE_LINK)
assert(IFSTATE.LINK_ADDR == InterfaceStatus.STATE_LINK_ADDR)
assert(IFSTATE.ADDR == InterfaceStatus.STATE_ADDR)
assert(InterfaceStatus.STATE_PINGING == 5)

STATUSES = { 
    InterfaceStatus.STATE_NO_INTERFACE : "Interface not found",
    InterfaceStatus.STATE_PLUGGED      : "Interface is down", 
    InterfaceStatus.STATE_UP           : "Interface is up with no link", 
    InterfaceStatus.STATE_LINK         : "Interface has a link with no address",
    InterfaceStatus.STATE_LINK_ADDR    : "Interface has a link but no IP address",
    InterfaceStatus.STATE_ADDR         : "Interface has an IP address but pings are failing",
    InterfaceStatus.STATE_PINGING      : "Interface is validated",
}

# FIXME May want to kill this at some point
import asmach as smach
smach.logdebug = lambda x: None

class Node:
    def __init__(self, *args, **kwargs):
        sigblock.save_mask()
        sigblock.block_signal(signal.SIGCHLD)
        rospy.init_node(*args, **kwargs)
        sigblock.restore_mask()
        rospy.core.add_shutdown_hook(self._shutdown_by_ros)
        reactor.addSystemEventTrigger('after', 'shutdown', self._shutdown_by_reactor)
        rospy.loginfo("Node __init__ done");

    def _shutdown_by_reactor(self):
        rospy.signal_shutdown("Reactor shutting down.")

    def _shutdown_by_ros(self, why):
        reactor.fireSystemEvent('shutdown')

class RoamNode:
    def __init__(self):
        Node("multi_interface_roam")
        self.interface_selector = interface_selector.InterfaceSelector()
        self.reconfig_server = dynamic_reconfigure.server.Server(MultiInterfaceRoamConfig, self.reconfigure)
        self._interfaces = self.interface_selector.interfaces.values()
        self.hostname = os.uname()[1]
        
        # Prepare topics to publish
        pub_namespace = rospy.remap_name('wifi')
        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray)
        self.ap_pub = rospy.Publisher(pub_namespace+"/accesspoint", AccessPoint)
        self.status_pub = rospy.Publisher(pub_namespace+"/status", MultiInterfaceStatus)
        self.iface_id_pub = rospy.Publisher(pub_namespace+'/current_iface_id', Int32, latch = True)
        self._wireless_interfaces = [ i for i in self._interfaces if i.__class__ == interface.WirelessInterface ]
        self.all_ap_pub = dict((iface, rospy.Publisher(pub_namespace+"/"+iface.iface+"/accesspoint", AccessPoint)) for iface in self._wireless_interfaces)
        
        # Kick off publication updates.
        self.interface_selector.update_event.subscribe_repeating(self._publish_status)

    def _publish_status(self):
        now = rospy.get_rostime()

        # current_iface_id
        ai = self.interface_selector.active_interfaces
        if not ai or ai[0] not in self._interfaces:
            index = -1
        else:
            index = self._interfaces.index(ai[0])
        self.iface_id_pub.publish(index)

        # accesspoint
        best_active = self.interface_selector.radio_manager.best_active
        for iface in self._wireless_interfaces:
            msg = self.gen_accesspoint_msg(iface)
            msg.header.stamp = now
            self.all_ap_pub[iface].publish(msg)
            if iface == best_active:
                self.ap_pub.publish(msg)
        if best_active is None:
            self.ap_pub.publish(AccessPoint())

        # status
        msg = MultiInterfaceStatus()
        for iface in self._interfaces:
            msg.interfaces.append(self.gen_status_msg(iface))
        self.status_pub.publish(msg)

        # diagnostics
        diags = []
        diags.append(('Tunnel Interface', self.interface_selector.tunnel_interface))
        if self.interface_selector.active_interfaces and \
             self.interface_selector.active_interfaces[0].score != self.interface_selector.TERRIBLE_INTERFACE:
            act_iface = self.interface_selector.active_interfaces[0]
            diags.append(('Active Interface', act_iface.iface ))
            diags += act_iface.diags
            if act_iface.goodness > 95:
                diag_summary = "Active interface %s running strong"%act_iface.iface
                diag_level = 0
            elif act_iface.goodness > 50:
                diag_summary = "Active interface %s is lossy"%act_iface.iface
                diag_level = 1
            else:
              if act_iface.goodness > 0:
                diag_summary = "Active interface %s is very poor"%act_iface.iface
              else:
                diag_summary = "Active interface %s is failing to ping"%act_iface.iface
              diag_level = 2
        else:
            diags.append(('Active Interface', "none"))
            diag_summary = 'No active interface'
            diag_level = 2
        ds = self.fill_diags("synthetic interface", diag_summary, self.hostname, diags)
        ds.level = diag_level
        statuses = [ds]

        for iface in self._interfaces:
            status = iface.status
            if status == InterfaceStatus.STATE_ADDR and iface.ping_loss < 100:
                status = InterfaceStatus.STATE_PINGING
                diag_summary = "Connected (goodness %f, reliability %f)"%(iface.goodness, iface.reliability)
            else:
                diag_summary = STATUSES[status]
            ds = self.fill_diags(iface.iface, diag_summary, self.hostname, iface.diags)
            statuses.append(ds)

        da = DiagnosticArray()
        da.header.stamp = rospy.get_rostime()
        da.status = statuses
        self.diag_pub.publish(da)
    
    
    @staticmethod
    def fill_diags(name, summary, hid, diags):
        ds = DiagnosticStatus()
        ds.values = [KeyValue(k, str(v)) for (k, v) in diags]
        ds.hardware_id = hid
        ds.name = rospy.get_caller_id().lstrip('/') + ": " + name
        ds.message = summary
        return ds
    
                
    @staticmethod
    def gen_status_msg(iface):
        msg = InterfaceStatus()
        msg.pretty_name = iface.prettyname
        msg.interface = iface.iface
        msg.state = iface.status
        if msg.state == InterfaceStatus.STATE_ADDR and iface.ping_loss < 100:
           msg.state = InterfaceStatus.STATE_PINGING
        msg.active_interface_rank = iface.rank
        msg.goodness = iface.goodness
        msg.reliability = iface.reliability
        msg.score = iface.score
        msg.prescore = iface.prescore
        msg.latency = iface.ping_latency
        msg.loss = iface.ping_loss
        if iface.__class__ == interface.WirelessInterface:
            cur_bss = iface.radio_sm.associated.get()
            if cur_bss:
                msg.bss = cur_bss
        return msg

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
        msg.channel = IEEE80211_Channels.get_channel(iface.wifi_frequency * 1e6)
        return msg

    def reconfigure(self, config, level):
        if config['interface'] not in self.interface_selector.interfaces:
            config['interface'] = ''
        interface = config['interface']
        ssid = config['ssid'] = config['ssid'][0:32]
        bssid = ""
        if not mac_addr.is_str(config['bssid']):
            config['bssid'] = ""
        else:
            bssid = mac_addr.str_to_packed(config['bssid'])
        if not self.interface_selector.tunnel_interface:
            config['use_tunnel'] = False
        use_tunnel = config['use_tunnel']

        self.interface_selector.set_mode(ssid, bssid, interface, use_tunnel, config['band'], config['scan_only'])

        return config

if __name__ == "__main__":
    def start():
        print "roam_node2 starting..."
        try:
            RoamNode()
        except:
            import traceback
            traceback.print_exc()
            print >> sys.stderr, "\nCaught exception during startup. Shutting down."
            reactor.fireSystemEvent('shutdown')
    reactor.addSystemEventTrigger('before', 'startup', start)
    reactor.run()
