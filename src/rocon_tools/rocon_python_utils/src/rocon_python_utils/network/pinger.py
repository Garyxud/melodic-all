#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: network.pinger
   :platform: Unix
   :synopsis: Tools built upon the system ping command.
"""
##############################################################################
# Imports
##############################################################################

from rocon_python_comms import WallRate

import rospy
import subprocess
import threading
import time

##############################################################################
# Helper class
##############################################################################


def mean(vals):
    return sum(vals) / len(vals)


def mdev(vals):
    mean_val = mean(vals)
    mean_diff = [abs(vals[i] - mean_val) for i in range(len(vals))]
    return mean(mean_diff)

##############################################################################
# Pinger class
##############################################################################


class Pinger(threading.Thread):
    '''
      The pinger class can run a threaded pinger at the desired frequency to
      check if a machine is available or not.

      **Usage:**

      .. code-block:: python

          from rocon_python_utils.network import Pinger

          pinger = Pinger('192.168.1.3', 0.2)
          pinger.start()
          # after some time
          print("Statistics: %s" % pinger.get_latency())  # [min,avg,max,mean deviation]
          print("Time since last seen %s" % pinger.get_time_since_last_seen())
    '''

    def __init__(self, ip, ping_frequency=0.2):
        """
        Initialises but doesn't start the thread yet. Use :func:`.run` to start
        pinging and collecting statistics.

        :param str ip:
        :param float ping_frequency: periodically ping at this frequency (Hz)
        """

        threading.Thread.__init__(self)
        self.daemon = True

        self.ip = ip
        self.ping_frequency = ping_frequency
        self.time_last_seen = time.time()

        # Keep track of the last 5 values
        self.buffer_size = 5
        self.buffer = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.values_available = 0
        self.current_ring_counter = 0

    def get_time_since_last_seen(self):
        """
          Time in seconds since this ip was last pingable.

          :returns: time (s) since seen
          :rtype: float
        """
        return time.time() - self.time_last_seen

    def get_latency(self):
        '''
          Latency statistics generated from the internal window (buffer).
          This is a ring buffer, so it will only use the most recent
          ping data.

          :returns: latency statistics (min, avg, max, mean deviation)
          :rtype: float[]
        '''
        if self.values_available == 0:
            return [0.0, 0.0, 0.0, 0.0]
        latency_stats = [min(self.buffer[:self.values_available]),
                         mean(self.buffer[:self.values_available]),
                         max(self.buffer[:self.values_available]),
                         mdev(self.buffer[:self.values_available])]
        return latency_stats

    def run(self):
        """
        Thread worker function - this does the actual pinging and records
        ping data in the ring buffers for processing into latency statistics
        as needed.
        """
        rate = WallRate(self.ping_frequency)
        while True:
            # In case of failure, this call will take approx 10s
            try:
                # Send 5 pings at an interval of 0.2s
                output = subprocess.check_output("ping -c 1 %s" % self.ip,
                                         shell=True, stderr=subprocess.STDOUT)
                self.time_last_seen = time.time()
                try:
                    parsed_output = \
                            output.splitlines()[-1].split(' ')[3].split('/')
                    latency_stats = [float(x) for x in parsed_output]
                    # Since this was a single ping, min = max = avg
                    self.buffer[self.current_ring_counter] = latency_stats[1]
                    self.values_available = self.values_available + 1 \
                            if self.values_available < self.buffer_size \
                            else self.buffer_size
                    self.current_ring_counter = \
                            (self.current_ring_counter + 1) % self.buffer_size

                except (KeyError, ValueError) as e:
                    # Had one occasion when something was wrong with ping output
                    rospy.logwarn("Unable to update latency statistics from " +
                                  self.ip + ". Error parsing ping output: " +
                                  str(e))
            except subprocess.CalledProcessError:
                # Ping failed. Do not update time last seen
                pass
            rate.sleep()
