#!/usr/bin/python

import roslib; roslib.load_manifest('map_laser')
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
from std_msgs.msg import ColorRGBA
import math


class MapLaser(object):
    def __init__(self):
        rospy.init_node('map_laser_filter')
        self.pub = rospy.Publisher('/base_scan_filter',
                                   LaserScan,
                                   queue_size=10)
        self.listener = tf.TransformListener()
        self.map = None
        self.save = None
        self.sub = rospy.Subscriber('/scan_filtered',
                                    LaserScan,
                                    self.laser_cb,
                                    queue_size=1)
        self.sub2 = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)


    def map_cb(self, msg):
        self.map = msg

    def get_laser_frame(self, msg):
        now = msg.header.stamp
        f2 = msg.header.frame_id
        f1 = '/map'
        self.listener.waitForTransform(f1, f2, now, rospy.Duration(.15))

        return self.listener.lookupTransform(f1, f2, now)

    def is_occupied(self, x, y):
        N = 2
        for dx in range(-N, N + 1):
            for dy in range(-N, N + 1):
                index = (x + dx) + (y + dy) * self.map.info.width
                if index < 0 or index > len(self.map.data):
                    continue
                value = self.map.data[index]
                if value > 50 or value < 0:
                    return True
        return False


    def laser_cb(self, msg):
        if self.map is None:
            return
        try:
            (trans, rot) = self.get_laser_frame(msg)
            self.save = (trans, rot)
        except Exception: # TODO should list handled exceptions
            if self.save is None:
                return
            (trans, rot) = self.save

        yaw = euler_from_quaternion(rot)[2]

        nr = []

        for (i, d) in enumerate(msg.ranges):
            if math.isnan(d) or d > msg.range_max or d < msg.range_min:
                nr.append(msg.range_max + 1.0)
                continue
            angle = yaw + msg.angle_min + msg.angle_increment*i
            dx = math.cos(angle) * d
            dy = math.sin(angle) * d
            map_x = trans[0] + dx
            map_y = trans[1] + dy

            grid_x = int((map_x - self.map.info.origin.position.x) / self.map.info.resolution)
            grid_y = int((map_y - self.map.info.origin.position.y) / self.map.info.resolution)

            if self.is_occupied(grid_x, grid_y):
                nr.append(msg.range_max + 1.0)
            else:
                nr.append(d)

        msg.ranges = nr
        self.pub.publish(msg)

if __name__ == '__main__':
    x = MapLaser()
    rospy.spin()
