#! /usr/bin/env python
import unittest
import numpy as np
import criutils as cu


class Test_array(unittest.TestCase):
  def test_unique(self):
    # Test random array of 100 points
    points = np.random.rand(100, 3)
    unique_points = cu.array.unique(points)
    self.assertEqual(points.shape, unique_points.shape)
    # Test repeated ones
    points = np.ones((100, 3))
    unique_points = cu.array.unique(points)
    self.assertEqual(len(unique_points), 1)
    # Test repeated zeros
    points = np.zeros((100, 3))
    unique_points = cu.array.unique(points)
    self.assertEqual(len(unique_points), 1)
