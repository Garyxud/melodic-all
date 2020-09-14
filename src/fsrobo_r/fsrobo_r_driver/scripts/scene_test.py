#!/usr/bin/env python

import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

class SceneInterface:
  def __init__(self):
    rospy.init_node('scene_test')
    self._psi = PlanningSceneInterface()
    self._base_frame = 'base_link'

  def add_box(self, name, size, pose, orientation=[0, 0, 0, 1]):
    pose = self.make_pose(pose, orientation)
    return self._psi.add_box(name, pose, size)

  def make_pose(self, pose, orientation):
    ps = PoseStamped()
    ps.header.frame_id = self._base_frame
    ps.pose.position.x = pose[0]
    ps.pose.position.y = pose[1]
    ps.pose.position.z = pose[2]
    ps.pose.orientation.x = orientation[0]
    ps.pose.orientation.y = orientation[1]
    ps.pose.orientation.z = orientation[2]
    ps.pose.orientation.w = orientation[3]

    return ps



if __name__ == '__main__':
  si = SceneInterface()

  rospy.sleep(1)
  #obj = si.add_box('my_box3', [2, 0.05, 1.16], [0, -0.55, 1.16/2-0.32])
  obj = si.add_box('wall1', [2, 0.05, 1.16], [2.0/2-0.65, -0.55, 1.16/2-0.32])
  obj = si.add_box('wall2', [0.05, 3, 1.16], [-0.65, 3.0/2-0.55, 1.16/2-0.32])
  obj = si.add_box('arm_mount1', [1, 0.2, 0.32], [0, 0, -0.32/2])
  obj = si.add_box('arm_mount2', [1, 0.2, 0.32], [0, 0.81, -0.32/2])
  obj = si.add_box('camera_mount1', [0.19, 0.19, 0.2], [-0.45, 0.2, -0.2/2])
  obj = si.add_box('camera_mount2', [0.19, 0.19, 0.2], [-0.45, 0.61, -0.2/2])
  obj = si.add_box('roomba', [0.45, 0.50, 0.23], [0, 0.40, 0.23/2-0.32])
  obj = si.add_box('floor', [2, 3, 0.01], [1-0.65, 3.0/2-0.55, -0.3205])
  