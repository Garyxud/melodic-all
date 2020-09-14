#! /usr/bin/env python

import time
import sys
import threading
from math import floor

import roslib; roslib.load_manifest('network_monitor_udp')
import rospy
from rospy import rostime

import actionlib
import sys
import network_monitor_udp.msg as msgs
from network_monitor_udp.udpmoncli import MonitorSource

DEFAULT_LATENCY_BINS=[.005, .01, .025, .05, .075, .1] 

class LinktestActionServer(object):
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.ActionServer(self._action_name, msgs.LinktestAction, self.start_linktest)
        self.max_source_id = 0

    def start_linktest(self, gh):
        self.max_source_id += 1
        linktest_thread = threading.Thread(target = self.linktest_thread_entry, args = (gh, self.max_source_id))
        linktest_thread.start()

    def linktest_thread_entry(self, gh, source_id): 
        feedback = msgs.LinktestFeedback()
        result   = msgs.LinktestResult()
    
        gh.set_accepted()
        goal = gh.get_goal()

        if not goal.ros_returnpath:
            source_id = None
    
        # init goal defaults
        unlimited_duration = (goal.duration == 0)
        if goal.update_interval == 0:
            goal.update_interval = goal.DEFAULT_UPDATE_INTERVAL
        if goal.bw == 0.0:
            goal.bw = goal.DEFAULT_BW
        if goal.bw_type == 0:
            goal.bw_type = goal.DEFAULT_BWTYPE
        if goal.latency_threshold == 0.0:
            goal.latency_threshold = goal.DEFAULT_LATENCY_THRESHOLD
        if goal.pktloss_threshold == 0.0:
            goal.pktloss_threshold = goal.DEFAULT_PKTLOSS_THRESHOLD
        if goal.pktsize == 0:
            goal.pktsize = goal.DEFAULT_PKTSIZE
        if len(goal.latencybins) == 0:
            goal.latencybins = DEFAULT_LATENCY_BINS

        rate = goal.bw / (goal.pktsize * 8)

        # start source
        source = MonitorSource(latencybins = goal.latencybins, destaddr = (goal.sink_ip, goal.sink_port), rate = rate, \
                               pkt_length = goal.pktsize, ros_returnpath = goal.ros_returnpath, roundtrip = goal.roundtrip, \
                               source_id = source_id, tos = goal.tos, rostopic_prefix = goal.rostopic_prefix, 
                               max_return_time = goal.max_return_time)

        adaptive_state = "fast_increase"
        bw_drop_count = 0
        max_known_good_bw = goal.bw

        start_time = time.time()
        next_time = start_time + goal.update_interval
        max_rtt = goal.latencybins[-1] + goal.max_return_time
        ignore_duration = (floor(max_rtt/goal.update_interval) + 1.0) * goal.update_interval
        feedback_start_time = start_time + ignore_duration * 1.1
        overall_stats = None

        while (unlimited_duration or time.time() - start_time < goal.duration) and \
                gh.get_goal_status().status == actionlib.GoalStatus.ACTIVE and not rospy.is_shutdown():
            sleep_time = next_time - time.time()
            if sleep_time > 0:
               time.sleep(sleep_time)
            curr_time = time.time()
            next_time = curr_time + goal.update_interval

            stats = source.get_statistics()

            for s in stats.exceptions:
                rospy.logwarn(s)

            hit_send_rate_limit = source.hit_send_rate_limit()

            if curr_time < feedback_start_time:
                continue

            if overall_stats is None:
                overall_stats = stats
            else:
                overall_stats.accumulate(stats)
           
            latency_histogram, latency, latency_restricted = stats.get_smart_bins()
            loss = 100 - 100 * sum(latency_histogram[0:-1])
            rx_bandwidth = stats.count * goal.pktsize * 8 / stats.get_window_length()

            # publish the feedback
            feedback.latency = latency_restricted
            feedback.loss = loss
            feedback.bandwidth = rx_bandwidth
            feedback.latency_histogram = latency_histogram
            feedback.stamp = rostime.Time(( stats.window_end + stats.window_start) / 2)
            gh.publish_feedback(feedback)

            rospy.logdebug("avg: %5.1f ms avgr: %5.1f ms loss: %6.2f", 1000*latency, 1000*latency_restricted, loss)

            if goal.bw_type == goal.BW_ADAPTIVE:
                tx_packets_in_window = int(goal.bw/(goal.pktsize * 8) * (stats.window_end - stats.window_start))

                all_metrics_ok = latency_restricted < goal.latency_threshold and loss < goal.pktloss_threshold \
                    and (rx_bandwidth > 0.8 * goal.bw or tx_packets_in_window - stats.count <= 3) and \
                    not hit_send_rate_limit

                if adaptive_state != "fast_decrease" and rx_bandwidth < 0.8 * goal.bw:
                    bw_drop_count += 1
                    if bw_drop_count == 3:
                        adaptive_state = "fast_decrease"
                else:
                    bw_drop_count = 0

                if adaptive_state == "fast_decrease":
                    if rx_bandwidth < 0.8 * goal.bw:
                        goal.bw = 0.8 * goal.bw
                        max_known_good_bw = rx_bandwidth
                    else:
                        adaptive_state = "slow_increase"
                        slow_increase_count = 1

                if adaptive_state == "fast_increase":
                    if all_metrics_ok:
                        max_known_good_bw = rx_bandwidth
                        goal.bw = 1.2 * rx_bandwidth
                    else:
                        adaptive_state = "slow_decrease"
                        goal.bw = max_known_good_bw

                if adaptive_state == "slow_decrease":
                    if all_metrics_ok:
                        adaptive_state = "slow_increase"
                        slow_increase_count = 1
                    else:
                        goal.bw = goal.bw * 0.99

                if adaptive_state == "slow_increase":
                    if all_metrics_ok:
                        if rx_bandwidth > max_known_good_bw or slow_increase_count > 5:
                            max_known_good_bw = goal.bw
                            adaptive_state = "fast_increase"
                        else:
                            slow_increase_count += 1
                            goal.bw = goal.bw * 1.01
                    else:
                        adaptive_state = "slow_decrease"

                if goal.bw < 2 * (goal.pktsize * 8 / goal.update_interval):
                    goal.bw = 2 * (goal.pktsize * 8 / goal.update_interval)
                
                source.set_tx_bandwidth(goal.bw)
                rospy.logdebug("Next TX bandwidth = %6.2f kbps", goal.bw/1000)

            rospy.logdebug("RX bandwidth = %6.2f kbps", rx_bandwidth/1000)

        source.shutdown()

        if overall_stats is not None:
            latency_histogram, latency, latency_restricted = overall_stats.get_smart_bins()
            loss = 100 - 100 * sum(latency_histogram[0:-1])
            rx_bandwidth = (overall_stats.count * goal.pktsize * 8) / (overall_stats.get_window_length())
            
            result.latency = latency_restricted
            result.loss = loss
            result.bandwidth = rx_bandwidth
            result.latency_histogram = latency_histogram

        if gh.get_goal_status().status == actionlib.GoalStatus.ACTIVE:
            gh.set_succeeded(result)
        else:
            gh.set_canceled(result)
 
if __name__ == "__main__":
    rospy.init_node('udpmonsourcenode')

    LinktestActionServer('performance_test')
    
    rospy.spin()

