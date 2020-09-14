import diagnostic_updater
import rospy
from fadecandy_msgs.msg import LEDArray

from .fadecandy_driver import FadecandyDriver


class FadecandyNode:
    def __init__(self):
        connection_retry_rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            try:
                self._driver = FadecandyDriver()
            except IOError as e:
                rospy.logwarn_once('Failed to connect to Fadecandy device: %s; will retry every second', e)
            else:
                rospy.loginfo('Connected to Fadecandy device')
                break
            connection_retry_rate.sleep()
        else:
            return

        self._set_leds_sub = rospy.Subscriber('set_leds', LEDArray, self._set_leds)

        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID(self._driver.serial_number)
        self._diagnostic_updater.add("Info", self._info_diagnostics)
        self._diagnostic_timer = rospy.Timer(rospy.Duration(1), lambda e: self._diagnostic_updater.force_update())

    def _info_diagnostics(self, stat):
        stat.summary(diagnostic_updater.DiagnosticStatus.OK, "OK")
        stat.add("Serial number", self._driver.serial_number)

    def _set_leds(self, led_array_msg):
        led_array_colors = []
        for led_strip_msg in led_array_msg.strips:
            led_strip_colors = [(c.r * 255, c.g * 255, c.b * 255) for c in led_strip_msg.colors]
            led_array_colors.append(led_strip_colors)

        # Convert to a list of r, g, b tuples and pass to the driver.
        self._driver.set_colors(led_array_colors)
