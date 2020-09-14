#! /usr/bin/env python

import roslib; roslib.load_manifest('network_monitor_udp')
from rospy import rostime
import rospy
import actionlib
import network_monitor_udp.msg as msgs
from network_monitor_udp.msg import LinktestGoal
import time
import math

class MetricLog:
    """
    Maintains a log of measurements for a variable. For each measurement, the
    time and value of the variable is recorded. Various statistics are computed 
    for the recorded data.
    """
    ALPHA = 0.1                   #: the alpha-parameter of the exponential average
    MOVING_STATS_SAMPLES = 10     #: default number of samples included in moving stats

    def __init__(self):        
        self.reset_all()

    def reset_statistics(self):
        """
        Resets the aggregated statistics.
        """
        self._min = 1e1000
        self._max = -1e1000
        self._avg = 0.0
        self._expavg = 0.0
        self._count = 0
        self._sum = 0.0
        self._curr = 0.0

    def reset_history(self):
        """
        Deletes all measurements recorded so far.
        """
        self._history = []
        
    def reset_all(self):
        """
        Resets measurement log and statistics.
        """
        self.reset_statistics()
        self.reset_history()

    def record(self, measurement, meas_time = None):
        """
        Records a measurement.
        
        @type measurement: float
        @param measurement: the value being recorded
        @type meas_time: float
        @param meas_time: the time of the measurement (if not specified the current time
        as retrieved by time.time() will be used)
        """
        if meas_time is None:
            meas_time = time.time()

        self._sum += measurement
        self._count += 1
        self._avg = self._sum / self._count
        self._max = max(self._max, measurement)
        self._min = min(self._min, measurement)
        if self._expavg == 0.0:
            self._expavg = measurement
        else:
            self._expavg = MetricLog.ALPHA * measurement + (1 - MetricLog.ALPHA) * self._expavg

        self._history.append((measurement, meas_time))
        self._curr = measurement

    def movavg(self, samples = MOVING_STATS_SAMPLES):
        """
        Returns the moving average of the recorded measurements.
        
        @type samples: int 
        @param samples: the number of samples in the moving average
        
        @rtype: float
        @return: moving average
        """
        try:
            return sum(rec[0] for rec in self._history[-samples:])/samples
        except IndexError:
            return 0.0

    def movstdev(self):
        """
        Returns the moving standard deviation of the recorded measurements.
        
        @type samples: int 
        @param samples: the number of samples in the moving standard deviation
        
        @rtype: float
        @return: moving standard deviation
        """
        try:
            return math.sqrt(sum([(x[0] - self.movavg())**2 for x in self._history[-MetricLog.MOVING_STATS_SAMPLES:]])/
                             (MetricLog.MOVING_STATS_SAMPLES - 1))
        except IndexError:
            return 0.0

    def stdev(self):
        """
        Returns the standard deviation of the recorded measurements.
        
        @rtype: float
        @return: standard deviation
        """
        try:
            return math.sqrt(sum([(x[0] - self._avg)**2 for x in self._history])/(self._count - 1))
        except ZeroDivisionError:
            return 0.0

    def empty(self):
        """
        @rtype: boolean
        @return: True if the measurement log is empty
        """
        return self._count == 0

    def min_time(self):
        """
        Returns the time of the first measurement recorded.

        @rtype: float
        @return: First measurement time or 0.0 if the log is empty.
        """
        try:
            return self._history[0][1]
        except IndexError:
            return 0.0

    def max_time(self):
        """
        Returns the time of the last measurement recorded.
        
        @rtype: float
        @return: Last measurement time or 0.0 if the log is empty.
        """
        try:
            return self._history[-1][1]
        except IndexError:
            return 0.0

    def duration(self):
        """
        Returns the period during which measurements where made (the time
        elapsed from the first to the last measurement).
        
        @rtype: float
        @return: duration in seconds
        """
        try:
            return self.max_time() - self.min_time()
        except IndexError:
            return 0.0

    def min(self):
        """
        @rtype: float
        @return: minimum value in the log
        """
        return self._min

    def max(self):
        """
        @rtype: float
        @return: maximum value in the log
        """
        return self._max

    def avg(self):
        """
        @rtype: float
        @return: the mean of the values in the log
        """
        return self._avg

    def expavg(self):
        """
        @rtype: float
        @return: the exponential average of the values in the log
        """
        return self._expavg

    def count(self):
        """
        @rtype: float
        @return: the number of values in the log
        """
        return self._count

    def curr(self):
        """
        @rtype: float
        @return: the last value recorded
        """
        return self._curr

    def history(self):
        """
        @rtype: [(float, float)]
        @return: the list of measurements recorded. Each measurement is a tuple (value, measurement_time).
        """        
        return self._history

class LinkTest:
    """
    This class implements a udpmon link test. It handles communication
    with the udpmonsourcenode via actionlib. Feedback information
    (bandwidth, loss, latency) is recorded into L{MetricLog} objects. The
    class has support for pre-empting a running test.
    """
    def __init__(self, name, goal, actionclient):
        """
        Builds a LinkTest object. The test will not be active until explicitly
        started with the L{start} function.

        @type name: string
        @param name: test name (used for logging information)
        @type goal: LinktestGoal
        @param goal: the goal object describing the link test
        @type actionclient: actionlib.ActionClient
        @param actionclient: the action client object
        """
        self.name = name
        self.goal = goal
        self.actionclient = actionclient
        self.custom_feedback_handler = None
    
        self.latency = MetricLog()
        """Latency measurement log. \n@type: L{MetricLog}"""
        self.loss = MetricLog()  
        """Packet loss measurement log. \n@type: L{MetricLog}"""
        self.bandwidth = MetricLog() 
        """Bandwidth measurement log. \n@type: L{MetricLog}"""

        self.overall_latency = None             #: Average latency over the test duration (available when test is done)
        self.overall_loss = None                #: Average loss over the test duration (available when test is done)
        self.overall_bandwidth = None           #: Average bandwidth over the test duration (available when test is done)
        self.overall_latency_histogram = None   #: Latency histogram of all packets received during the test (available when test is done)

        self.started = False              #: True if test was started
        self.done = False                 #: True if test has finished (or was preempted succesfully)

    def start(self):
        """
        Starts the test.
        """
        self.goal_handle = self.actionclient.send_goal(self.goal, self._handle_link_transition, self._handle_link_feedback)
        self.started = True

    def stop(self):
        """
        Stops (preempts) a running test.
        """
        if self.started:
            self.goal_handle.cancel()

    def reset_statistics(self):
        """ 
        Resets bandwidth, latency and loss statistics.
        """
        self.latency.reset_all()
        self.loss.reset_all()
        self.bandwidth.reset_all()

    def _handle_link_transition(self, goal_handle):
        rospy.logdebug("received link transition")
        if goal_handle.get_comm_state() == actionlib.CommState.DONE:
            rospy.logdebug("goal transitioned to done state")
            self.done = True

            result = goal_handle.get_result()
            if result is None:
                return

            self.overall_latency = result.latency
            self.overall_loss = result.loss
            self.overall_bandwidth = result.bandwidth
            self.latency_histogram = result.latency_histogram

            rospy.logdebug("[FINAL RESULT %s]: Avg latency: %.1f ms Avg loss: %.2f Avg bandwidth: %.3f", \
                            self.name, result.latency*1000, result.loss, result.bandwidth/1000)

    def _handle_link_feedback(self, gh, feedback):
        if self.custom_feedback_handler is not None:
            self.custom_feedback_handler(gh, feedback)

        self.latency.record(feedback.latency, feedback.stamp.to_sec())
        self.loss.record(feedback.loss, feedback.stamp.to_sec())
        self.bandwidth.record(feedback.bandwidth, feedback.stamp.to_sec())
        rospy.logdebug("[FEEDBACK %s]: Latency %.1f ms Loss %.2f Bandwidth %.3f Kbps", 
                       self.name, feedback.latency*1000, feedback.loss, feedback.bandwidth/1000)        

    def linkdown(self):
        """
        @rtype: boolean
        @return: True if the link is currently down (last measured bandwidth is close to zero).
        """
        return self.bandwidth.curr() < 0.00001

    def set_custom_feedback_handler(self, custom_feedback_handler):
        """
        Sets a custom feedback handler.
        
        @type custom_feedback_handler: func(GoalHandle, LinktestFeedback)
        @param: the custom feedback handler that gets called whenever feedback is received
        """
        self.custom_feedback_handler = custom_feedback_handler


class UdpmonsourceHandle :
    """
    This class maintains a handle to a L{udpmonsourcenode.py} action server. 
    It is used to create link tests that have as source a particular C{udpmonsourcenode.py} node.
    """
 
    def __init__(self, action_name = "performance_test"):
        """
        Creates an L{actionlib.ActionClient} object and waits for connection to the action server. 

        @raise Exception: if the connection does not succeed
        
        @type action_name: str
        @param action_name: action server name, by default this is C{"performance_test"}, but if the action
        server lives in a different namespace than the full path should be given 
        (e.g. C{"/source2/performance_test"})
        """
        self.actionclient = actionlib.ActionClient(action_name, msgs.LinktestAction)
        self.count = 0
        started = self.actionclient.wait_for_server() # wait until the udpmonsourcenode has started up
        if started is False:
            raise Exception, "could not connect to action server"

    def _generate_name(self, action_name):
        count = self.count
        self.count += 1
        return ("%s_%04d")%(action_name, self.count)

    def create_test(self, name = None, **kwargs):
        """
        Creates and returns a link test. The link test will need to be started with its C{start()} function.
        
        @type name: str
        @param name: an optional test name. This name will be used for log mesages.
        
        @type kwargs: dict
        @param kwargs: a dictionary of test parameters. These are the .action file parameters. See the wiki page
        at U{http://www.ros.org/wiki/network_monitor_udp} for a full description.
        """
        if name is None:
            name = self._generate_name("bwtest")
        goal = msgs.LinktestGoal(**kwargs)
        test = LinkTest(name, goal, self.actionclient)
        return test
    
    def cancel_all_tests(self):
        """
        Cancels all currently running tests. As a precaution, this function should be 
        called at the beginning of any test "session" as there may be orphan link tests
        still being run from the C{udpmonsourcenode.py} node, if that node has not been restarted.
        """
        self.actionclient.cancel_all_goals()
        # to prevent a race condition betwen creating a new goal and
        # the cancel_all_goals request
        time.sleep(2)

    def get_link_capacity(self, **kwargs):
        """
        This is a convenience function that creates an adaptive bandwidth test in order to saturate
        and determine a link's capacity. By default, the parameters of the test are

        - one-way test (not roundtrip)
        - uses ROS for return path instead of UDP
        - update_interval: 0.2 seconds
        - packet loss threshold: 0.5%
        - latency threshold: 30ms
        - initial bandwidth: 0.5Mbit/s
        - maximum duration: 30 seconds
        
        This function should return faster than the specified timeout if the conditions for link saturation
        are fulfilled (i.e. it hits a bandwidth ceiling for a specified period of time). 

        @type kwargs: dict
        @param kwargs: a list of test parameters

        @rtype: float
        @return: the link capacity if it could be determined or 0.0, otherwise.
        """
        name = self._generate_name("link_capacity_test")
        test_params = { "update_interval" : 0.2, "ros_returnpath" : True, "bw" : 0.5*10**6,
                        "bw_type" : LinktestGoal.BW_ADAPTIVE, 
                        "latency_threshold" : 30e-3, "pktloss_threshold" : 0.5, "duration": 30.0 }
        test_params.update(kwargs)
        goal = msgs.LinktestGoal(**test_params)
        test = LinkTest(name, goal, self.actionclient)
        bwlog = test.bandwidth
        
        link_capacity = 0.0
        test.start()
        while not test.done:
            time.sleep(0.5)
            if bwlog.duration() < 5.0:
                continue               
            if bwlog.max() < 0.0001:
                break
            if bwlog.curr() / bwlog.max() > 0.5 and bwlog.curr() / bwlog.expavg() < 1.0:
                link_capacity = bwlog.expavg()
                break
        if not test.done:
            test.stop()

        return link_capacity
