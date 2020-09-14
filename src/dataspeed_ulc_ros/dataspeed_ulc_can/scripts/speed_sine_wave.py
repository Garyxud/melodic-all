#! /usr/bin/env python
import rospy
import math
from dataspeed_ulc_can import Speed
from dataspeed_ulc_msgs.msg import UlcReport


class SpeedSineWave(Speed):
    APPROACHING = 0
    TRACKING = 1

    def __init__(self):
        rospy.init_node('speed_sine_wave')
        super(SpeedSineWave, self).__init__()

        self.speed_meas = 0
        self.reached_target_stamp = -1
        self.state = self.APPROACHING
        rospy.Subscriber('/vehicle/ulc_report', UlcReport, self.recv_report)

    def timer_callback(self, event):

        if not self.enabled:
            self.t = 0
            self.state = self.APPROACHING
            return

        if self.state == self.APPROACHING:
            self.ulc_cmd.linear_velocity = self.v1
            self.t = 0
            if abs(self.ulc_cmd.linear_velocity - self.speed_meas) < 0.4 and self.reached_target_stamp < 0:
                self.reached_target_stamp = event.current_real.to_sec()

            # Wait 3 seconds before starting the sine wave input
            if self.reached_target_stamp > 0 and (event.current_real.to_sec() - self.reached_target_stamp) > 3:
                self.state = self.TRACKING
                self.reached_target_stamp = -1

        elif self.state == self.TRACKING:
            amplitude = 0.5 * (self.v2 - self.v1)
            offset = 0.5 * (self.v2 + self.v1)
            self.ulc_cmd.linear_velocity = offset - amplitude * math.cos(2 * math.pi / self.period * self.t)
            self.t += 0.02

        self.pub_ulc_cmd.publish(self.ulc_cmd)
        if self.v1 == 0 and self.v2 == 0:
            rospy.logwarn_throttle(1.0, 'both speed targets are zero')

    def recv_report(self, msg):
        self.speed_meas = msg.speed_meas


if __name__ == '__main__':
    try:
        node_instance = SpeedSineWave()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
