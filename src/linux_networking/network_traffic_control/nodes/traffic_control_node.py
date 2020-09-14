#! /usr/bin/env python

from subprocess import call, STDOUT
import shlex 
import math

import roslib; roslib.load_manifest('network_traffic_control')
import rospy

import dynamic_reconfigure.server
from network_traffic_control.cfg import TrafficControlConfig

DEFAULT_PACKET_SIZE = 1500
PACKET_QUEUE_SIZE = 1
MAX_PACKET_BURST = 5

class TrafficControlManager:
    def __init__(self, interface, 
                 interface_ifb = "ifb0", 
                 filter_ingress = "u32 match u32 0 0",
                 filter_egress = "u32 match u32 0 0"):
       self.interface = interface
       self.interface_ifb = interface_ifb
       self.filter_egress = filter_egress
       self.filter_ingress = filter_ingress
       self.ifb_not_set_up = True
       self.devnull = open('/dev/null', 'w')
       self.config = dict()
       self.reset_egress()
       self.reset_ingress()

    def reset_egress(self):
        call(shlex.split("tc qdisc del dev " + self.interface + " root"),
             stdout = self.devnull, stderr = STDOUT)
        self.config['bandwidth_egress'] = 0.0
        self.config['latency_egress'] = 0.0
        self.config['loss_egress'] = 0.0
        self.egress_control = False

    def init_egress(self):
        # htb qdisc
        ret = call(shlex.split("tc qdisc add dev " + self.interface + " handle 1: root htb"), 
                   stdout = self.devnull, stderr = STDOUT)
        if ret != 0:
            self.config['status'] = "FAIL"
            self.config['errmsg'] = "Failed to add htb qdisc at root on " + self.interface
            self.reset_egress()
            return 
        # htb class
        ret = call(shlex.split("tc class add dev " + self.interface + " parent 1: classid 1:1"
                               " htb rate 10000Mbps"), 
                   stdout = self.devnull, stderr = STDOUT)
        if ret != 0:
            self.config['status'] = "FAIL"
            self.config['errmsg'] = "Failed to add htb class on " + self.interface
            self.reset_egress()
            return 
        # filter
        ret = call(shlex.split("tc filter add dev " + self.interface + " protocol ip prio 1 " +
                               self.filter_egress + " flowid 1:1"),
                   stdout = self.devnull, stderr = STDOUT)
        if ret != 0:
            self.config['status'] = "FAIL"
            self.config['errmsg'] = "Failed to set egress filter on " + self.interface
            self.reset_egress()
            return 
        # tbf qdisc
        ret = call(shlex.split("tc qdisc add dev " + self.interface + " parent 1:1 handle 2:1 "  
                               "tbf rate 10000mbit buffer 500000 limit 100000"),
                   stdout = self.devnull, stderr = STDOUT)
        if ret != 0:
            self.config['status'] = "FAIL"
            self.config['errmsg'] = "Failed to add tbf qdisc on " + self.interface
            self.reset_egress()
            return 
        # enable egress control
        self.egress_control = True

    def set_rules(self, config, direction):
        if direction == "egress":
            latency_new = config['latency_egress']
            loss_new = config['loss_egress']
            bandwidth_new = config['bandwidth_egress']
            
            latency_old = self.config['latency_egress']
            loss_old = self.config['loss_egress']
            bandwidth_old = self.config['bandwidth_egress']
            
            interface = self.interface
        elif direction == "ingress":
            latency_new = config['latency_ingress']
            loss_new = config['loss_ingress']
            bandwidth_new = config['bandwidth_ingress']
            
            latency_old = self.config['latency_ingress']
            loss_old = self.config['loss_ingress']
            bandwidth_old = self.config['bandwidth_ingress']

            interface = self.interface_ifb
        else:
            self.config['status'] = "FAIL"
            self.config['errmsg'] = "Unknown direction: " + direction

        latency_loss_disabled = (latency_new == 0.0 and loss_new == 0.0) and \
            (latency_old != 0.0 or loss_old != 0.0) 
        latency_loss_enabled = (latency_new != 0.0 or loss_new != 0.0) and \
            (latency_old == 0.0 and loss_old == 0.0) 

        if bandwidth_new != bandwidth_old or \
                config['packet_size'] != self.config['packet_size'] or latency_loss_disabled:
            if bandwidth_new > 0.0:
                bw_kbits = bandwidth_new/1e3
                bw_limit = (config['packet_size'] + 100) * PACKET_QUEUE_SIZE 
                bw_buffer = (config['packet_size'] + 100) * MAX_PACKET_BURST
            else:
                bw_kbits = 10000 * 1e3 # 10Gbits
                bw_limit = 100000
                bw_buffer = 500000

            ret = call(shlex.split("tc qdisc change dev " + interface + " handle 2:1 tbf "  
                                   "rate %.2fkbit buffer %d limit %d"%(bw_kbits, bw_buffer, bw_limit)),                   
                       stdout = self.devnull, stderr = STDOUT)

            if ret != 0:
                self.config['status'] = "FAIL"
                self.config['errmsg'] = self.config['errmsg'] + \
                    "Failed to change tbf (bandwidth) rule on " + interface
                if direction == "egress":
                    self.reset_egress()
                else:
                    self.reset_ingress()
                return 

            netem_action = "add"
        else:
            if not latency_loss_enabled:
                netem_action = "change"
            else:
                netem_action = "add"

        # touch the netem rule (add or change it) only if latency or loss are not zero
        # and either they have changed since last update or the tbf rule has changed
        if (latency_new != 0.0 or loss_new != 0.0) and \
                (netem_action == "add" or latency_new != latency_old or loss_new != loss_old):
            latency_ms = 1e3 * latency_new
            loss = loss_new
            if bandwidth_new == 0.0:
                limit = 1000
            else:
                limit = math.ceil(1.0 + 
                                  latency_new / (8.0 * float(config['packet_size'])/bandwidth_new))
                limit = int(limit)

            ret = call(shlex.split("tc qdisc " + netem_action + " dev " +  interface + 
                                   " parent 2:1 handle 3:1 "
                                   "netem latency %.2fms loss %.2f%% limit %d"%(latency_ms, loss, limit)),
                       stdout = self.devnull, stderr = STDOUT)
            if ret != 0:
                self.config['status'] = "FAIL"
                self.config['errmsg'] = "Failed to " + netem_action + " netem qdisc on " + interface
                if direction == "egress":
                    self.reset_egress()
                else:
                    self.reset_ingress()
                return 

        if direction == "egress":
            self.config['bandwidth_egress'] = bandwidth_new
            self.config['latency_egress'] = latency_new
            self.config['loss_egress'] = loss_new
        else:
            self.config['bandwidth_ingress'] = bandwidth_new
            self.config['latency_ingress'] = latency_new
            self.config['loss_ingress'] = loss_new

    def reset_ingress(self):
        ret = call(shlex.split("tc qdisc del dev " + self.interface + " ingress"), 
                   stdout = self.devnull, stderr = STDOUT)
        ret = call(shlex.split("tc qdisc del dev " + self.interface_ifb + " root"), 
                   stdout = self.devnull, stderr = STDOUT)
        self.config['bandwidth_ingress'] = 0.0
        self.config['loss_ingress'] = 0.0
        self.config['latency_ingress'] = 0.0
        self.ingress_control = False

    def init_ingress(self):
        # set up ifb device
        if self.ifb_not_set_up:
            call(shlex.split("modprobe ifb"),
                 stdout = self.devnull, stderr = STDOUT)
            call(shlex.split("ip link set dev " + self.interface_ifb + " up"),
                 stdout = self.devnull, stderr = STDOUT)
            self.ifb_not_set_up = False
        # add ingress qdisc
        ret = call(shlex.split("tc qdisc add dev " + self.interface + " handle ffff: ingress"),
                   stdout = self.devnull, stderr = STDOUT)
        if ret != 0:
            self.config['status'] = "FAIL"
            self.config['errmsg'] = self.config['errmsg'] + \
                "Failed to add ingress qdisc at root on " + self.interface
            self.reset_ingress()
            return 
        # add redirect filter
        ret = call(shlex.split("tc filter add dev " + self.interface + " parent ffff: protocol ip " +
                               self.filter_ingress + 
                               " action mirred egress redirect dev " + self.interface_ifb),
                   stdout = self.devnull, stderr = STDOUT)
        if ret != 0:
            self.config['status'] = "FAIL"
            self.config['errmsg'] = self.config['errmsg'] + \
                "Failed to add redirect filter on ingress for " + self.interface
            self.reset_ingress()
            return 
        # add tbf root rule on ifb-device
        ret = call(shlex.split("tc qdisc add dev " + self.interface_ifb + " root handle 2:1 " +
                               "tbf rate 10000mbit buffer 500000 limit 100000"),
                   stdout = self.devnull, stderr = STDOUT)
        if ret != 0:
            self.config['status'] = "FAIL"
            self.config['errmsg'] = self.config['errmsg'] + \
                "Failed to add tbf qdisc at root on " + self.interface_ifb
            self.reset_ingress()
            return 
        # enable ingress control
        self.ingress_control = True

    def reconfigure(self, config, level):
        self.config['status'] = 'OK'
        self.config['errmsg'] = ''

        if 'packet_size' not in self.config:
            self.config['packet_size'] = config['packet_size']

        if 1 & level or 4 & level:
            if self.egress_control and config['bandwidth_egress'] == 0.0 and \
                    config['latency_egress'] == 0.0 and config['loss_egress'] == 0.0:
                self.reset_egress()

            if not self.egress_control and (config['bandwidth_egress'] != 0.0 or \
                    config['latency_egress'] != 0.0 or config['loss_egress'] != 0.0):
                self.init_egress()
                
            if self.egress_control:
                self.set_rules(config, "egress")
                
        if 2 & level or 4 & level:
            if self.ingress_control and config['bandwidth_ingress'] == 0.0 and \
                    config['latency_ingress'] == 0.0 and config['loss_ingress'] == 0.0:
                self.reset_ingress()

            if not self.ingress_control and (config['bandwidth_ingress'] != 0.0 or \
                    config['latency_ingress'] != 0.0 or config['loss_ingress'] != 0.0):
                self.init_ingress()
                
            if self.ingress_control:
                self.set_rules(config, "ingress")

        self.config['packet_size'] = config['packet_size']

        return self.config
  

if __name__ == "__main__":
    rospy.init_node("traffic_control_node")

    interface = rospy.get_param("~interface")
    args = dict()
    try:
        args["interface_ifb"] = rospy.get_param("~interface_ifb")        
    except KeyError:
        pass
    try:
        args["filter_egress"] = rospy.get_param("~filter_egress")        
    except KeyError:
        pass
    try:
        args["filter_ingress"] = rospy.get_param("~filter_ingress")        
    except KeyError:
        pass

    tc_mgr = TrafficControlManager(interface, **args)

    try:
        dynamic_reconfigure.server.Server(TrafficControlConfig, tc_mgr.reconfigure)
        rospy.spin()
    finally:
        tc_mgr.reset_egress()
        tc_mgr.reset_ingress()
