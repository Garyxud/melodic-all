#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: wall_rate
   :platform: Unix
   :synopsis: Wall time rate class.


This module provides a wall time rate class (missing in rospy).

----

"""

##############################################################################
# Imports
##############################################################################

import rospy
import time

##############################################################################
# Wall Rate (Uses system time, independent of ros /clock)
##############################################################################


class WallRate():
    """
    A wall time implementation of ros' rospy.time.Rate class.

    Usage:

    .. code-block:: python

        from rocon_python_comms import WallRate

        rate = WallRate(10)  # 10Hz = 0.1s period
        while True:
            # do something
            rate.sleep()
    """

    def __init__(self, rate):
        '''
          :param float rate: rate in hertz, if rate = 0, then won't sleep
        '''
        self.rate = rate
        self.period = 1.0 / rate if rate > 0.0 else 0.0
        self.recorded_time = time.time()

    def sleep(self):
        """
        Sleep until the rate period is exceeded.
        """
        current_time = time.time()
        elapsed = current_time - self.recorded_time
        if self.period - elapsed > 0:
            rospy.rostime.wallsleep(self.period - elapsed)
        self.recorded_time = time.time()
