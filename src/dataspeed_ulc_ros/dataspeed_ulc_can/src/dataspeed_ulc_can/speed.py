#! /usr/bin/env python
import rospy
from std_msgs.msg import Bool
from dataspeed_ulc_msgs.msg import UlcCmd


# This is a base class that is inherited by SpeedSquareWave and SpeedSineWave
class Speed(object):

    def __init__(self):
        rospy.Timer(rospy.Duration(0.02), self.timer_callback)

        self.t = 0
        self.enabled = False

        self.ulc_cmd = UlcCmd()
        self.ulc_cmd.enable_pedals = True
        self.ulc_cmd.enable_steering = False
        self.ulc_cmd.shift_from_park = False

        # Topics
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.recv_enable)
        self.pub_ulc_cmd = rospy.Publisher('/vehicle/ulc_cmd', UlcCmd, queue_size=1)

        # Parameters
        self.v1 = rospy.get_param('~v1', default=0.0)  # Speed 1
        self.v2 = rospy.get_param('~v2', default=5.0)  # Speed 2
        self.period = rospy.get_param('~period', default=10.0) # Period of wave pattern
        self.ulc_cmd.enable_shifting = rospy.get_param('~enable_shifting', default=False)  # Enable shifting between non-Park gears
        self.ulc_cmd.linear_accel = rospy.get_param('~accel_limit', default=0.0)  # Override default acceleration limit
        self.ulc_cmd.linear_decel = rospy.get_param('~decel_limit', default=0.0)  # Override default acceleration limit

    def timer_callback(self, event):
        # Implemented in derived classes
        pass

    def recv_enable(self, msg):
        if msg.data and not self.enabled:
            self.t = 0

        self.enabled = msg.data
