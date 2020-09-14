#! /usr/bin/env python
import unittest
import numpy as np
import baldor as br
import criutils as cu
# Messages
from geometry_msgs.msg import (Point, Quaternion, Pose, Vector3, Transform,
                                                                        Wrench)
from sensor_msgs.msg import RegionOfInterest


class Test_conversions(unittest.TestCase):
  def test_from_dict(self):
    transform_dict = dict()
    for _ in range(100):
      T0 = br.transform.random()
      transform_dict['rotation'] = br.transform.to_quaternion(T0).tolist()
      transform_dict['translation'] = T0[:3,3].tolist()
      T1 = cu.conversions.from_dict(transform_dict)
      self.assertTrue(br.transform.are_equal(T0, T1))

  def test_point(self):
    for _ in range(100):
      p0 = np.random.sample(3)
      point_msg = cu.conversions.to_point(p0)
      p1 = cu.conversions.from_point(point_msg)
      np.testing.assert_allclose(p0, p1)

  def test_pose(self):
    for _ in range(100):
      T0 = br.transform.random()
      pose_msg = cu.conversions.to_pose(T0)
      T1 = cu.conversions.from_pose(pose_msg)
      self.assertTrue(br.transform.are_equal(T0, T1))

  def test_quaternion(self):
    for _ in range(100):
      q0 = br.quaternion.random()
      quat_msg = cu.conversions.to_quaternion(q0)
      q1 = cu.conversions.from_quaternion(quat_msg)
      self.assertTrue(br.quaternion.are_equal(q0, q1))

  def test_roi(self):
    # from_roi
    roi_msg = RegionOfInterest()
    roi_msg.x_offset = np.random.randint(1280/2)
    roi_msg.y_offset = np.random.randint(1024/2)
    roi_msg.width = np.random.randint(1280/2)
    roi_msg.height = np.random.randint(1024/2)
    roi_np = cu.conversions.from_roi(roi_msg)
    self.assertEqual(type(roi_np), list)
    self.assertEqual(len(roi_np), 2)
    # to_roi
    top_left = 200*np.random.sample(2)
    bottom_right = 800*np.random.sample(2) + 220
    roi_msg = cu.conversions.to_roi(top_left, bottom_right)
    self.assertEqual(type(roi_msg), RegionOfInterest)

  def test_transform(self):
    for _ in range(100):
      T0 = br.transform.random()
      tf_msg = cu.conversions.to_transform(T0)
      T1 = cu.conversions.from_transform(tf_msg)
      self.assertTrue(br.transform.are_equal(T0, T1))

  def test_vector3(self):
    for _ in range(100):
      v0 = np.random.sample(3)
      vect_msg = cu.conversions.to_vector3(v0)
      v1 = cu.conversions.from_vector3(vect_msg)
      np.testing.assert_allclose(v0, v1)

  def test_wrench(self):
    for _ in range(100):
      w0 = np.random.sample(6)
      wrench_msg = cu.conversions.to_wrench(w0)
      w1 = cu.conversions.from_wrench(wrench_msg)
      np.testing.assert_allclose(w0, w1)
