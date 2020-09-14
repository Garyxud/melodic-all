#! /usr/bin/env python

from __future__ import with_statement

import roslib; roslib.load_manifest('network_monitor_udp')
import rospy

import sys
import network_monitor_udp.msg as msgs
import threading
import time
import traceback
import threading
import subprocess
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray

inf = 1e1000 

class NetworkWatchdog:
    def __init__(self, topic, timeout, max_latency, max_loss, action, action_name):
        self.action = action
        self.timeout = timeout
        self.max_latency = max_latency
        self.max_loss = max_loss
        self.action_name = action_name

        self.start_time = rospy.get_time()
        self.prev_stamp = self.start_time 
        self.trigger_count = 0
        self.outage_count = 0
        self.outage_length = 0
        self.longest_outage = 0
        self.longest_outage_time = None 
        self.latest_trigger_time = None
        self.longest_action_duration = -inf
        self.longest_action_time = None

        self.interruption_time = inf
        self.interruption_time_orig = inf
        self.action_running = False
        self.outage_in_progress = False

        self.mutex = threading.Lock()

        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray)
        rospy.Subscriber(topic, msgs.UdpMonitor, self.callback)

    def callback(self, msg):
        sum = 1
        i = 0
        while i < len(msg.latency_bin_limits) and msg.latency_bin_limits[i] <= self.max_latency:
            sum -= msg.latency_bin_values[i]
            i += 1
    
        stamp = msg.header.stamp.to_sec()

        trigger = False

        with self.mutex:
            if sum < self.max_loss:
                if self.outage_in_progress:
                    self.outage_in_progress = False
                    self.interruption_time = inf
                    rospy.loginfo("Network is back up. Outage length %.1f s."%(stamp - self.interruption_time_orig))
            else:
                if not self.outage_in_progress:
                    self.outage_in_progress = True 
                    rospy.logwarn("Network outage detected.")
                    self.interruption_time = self.prev_stamp
                    self.interruption_time_orig = self.prev_stamp
                    self.outage_count += 1
                if stamp - self.interruption_time > self.timeout and not self.action_running:
                    self.action_running = True
                    self.trigger_count += 1
                    trigger = True
                self.outage_length = stamp - self.interruption_time_orig
                if self.outage_length > self.longest_outage:
                    self.longest_outage = self.outage_length
                    self.longest_outage_time = self.interruption_time_orig
            self.prev_stamp = stamp

        if trigger:
            self.latest_trigger_time = rospy.get_time()
            rospy.logerr("Network watchdog action triggered: %s", self.action_name)            
            self.diagnostics() # Print those diagnostics right away.
            threading.Thread(target=self.do_action).start()

    def do_action(self):
        try:
            self.action()
        except Exception:
            print "Caught exception during action."
            traceback.print_exc(10)
        now = rospy.get_time()
        with self.mutex:  
            if self.interruption_time != inf: 
                self.interruption_time = now
            self.action_running = False
            duration = now - self.latest_trigger_time
            if self.longest_action_duration < duration:
                self.longest_action_duration = duration
                self.longest_action_time = self.latest_trigger_time

    def diagnostics(self):
        ds = DiagnosticStatus()
        ds.name = rospy.get_caller_id().lstrip('/') + ": " + "Network Watchdog"
        ds.hardware_id = "none"
 
        with self.mutex:
            now = rospy.get_time()
            time_to_timeout = self.timeout - (now - self.interruption_time)
            elapsed_orig = now - self.interruption_time_orig
            
            # Parameters
            ds.values.append(KeyValue("Timeout (s)", "%.1f"%self.timeout))
            ds.values.append(KeyValue("Latency threshold (ms)", str(int(self.max_latency * 1000))))
            ds.values.append(KeyValue("Loss threshold (%)", "%.1f"%(self.max_loss * 100)))
            ds.values.append(KeyValue("Timeout command", str(self.action_name)))
            
            # Network outage stats
            ds.values.append(KeyValue("Network is down", str(self.outage_in_progress)))
            ds.values.append(KeyValue("Outage count", str(self.outage_count)))
            if self.outage_in_progress:
                ds.values.append(KeyValue("Time since outage start (s)", "%.1f"%elapsed_orig))
            if self.outage_length:
                ds.values.append(KeyValue("Latest outage length", "%.1f"%self.outage_length))
                ds.values.append(KeyValue("Longest outage length (s)", "%.1f"%self.longest_outage))
           
            # Command stats 
            ds.values.append(KeyValue("Timeout command in progress", str(self.action_running)))
            ds.values.append(KeyValue("Trigger count", str(self.trigger_count)))
            if self.outage_in_progress:
                if not self.action_running:
                    ds.values.append(KeyValue("Time to timeout (s)", "%.1f"%(time_to_timeout)))
            if self.action_running:
                ds.values.append(KeyValue("Time since trigger", str(now - self.latest_trigger_time)))
            if self.longest_action_time:
                ds.values.append(KeyValue("Longest timeout command run time", "%.1f"%self.longest_action_duration))
            
            # Stats
            ds.values.append(KeyValue("Node start time", time.ctime(self.start_time)))
            if self.interruption_time_orig != inf:
                ds.values.append(KeyValue("Latest outage start time", time.ctime(self.interruption_time_orig)))
            if self.longest_outage_time:
                ds.values.append(KeyValue("Longest outage start time", time.ctime(self.longest_outage_time)))
            if self.latest_trigger_time:
                ds.values.append(KeyValue("Latest trigger time", time.ctime(self.latest_trigger_time)))
            if self.longest_action_time:
                ds.values.append(KeyValue("Time of longest timeout command", time.ctime(self.longest_action_time)))
            
            # Summary
            if self.interruption_time == inf:
                ds.message = "Network is up"
                ds.level = DiagnosticStatus.OK
                if self.action_running:
                    ds.message += "; Timeout command in progress (%.0f s)"%(now - self.latest_trigger_time)
            else:
                if self.action_running:
                    ds.message = "Timeout command in progress (%.0f s; %.0f s)"%(elapsed_orig, now - self.latest_trigger_time)
                    ds.level = DiagnosticStatus.ERROR
                else:
                    ds.message = "Network is down (%.0f s; %.0f s)"%(elapsed_orig, time_to_timeout)
                    ds.level = DiagnosticStatus.WARN

        da = DiagnosticArray()
        da.header.stamp = rospy.get_rostime()
        da.status.append(ds)
        self.diag_pub.publish(da)
        
class Action:
    def __init__(self, command):
        self.command = command

    def run(self):
        subprocess.call(['/bin/sh', '-c', self.command], stdin = subprocess.PIPE)

def main():
    try:
    	rospy.init_node('udp_monitor_watchdog', anonymous=True)

        timeout = rospy.get_param("~timeout", 120)
        max_latency = rospy.get_param("~max_latency", 0.5)
        max_loss = rospy.get_param("~max_loss", 0.95)
        command = rospy.get_param("~command", "true")

        nw = NetworkWatchdog('udpmonitor', timeout, max_latency, max_loss, Action(command).run, command)

        sleeptime = rospy.Duration(1)
        while not rospy.is_shutdown():
            try:
                nw.diagnostics()
            except:
                # Don't want the network watchdog to die because of a diagnostic exception.
                traceback.print_exc(10)
            rospy.sleep(sleeptime)
    except KeyboardInterrupt:
        pass
    if nw.action_running:
        print "Waiting for trigger command to complete."

if __name__ == "__main__":
    main()
