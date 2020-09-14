#! /usr/bin/env python
import unittest
import numpy as np
import criutils as cu


class Test_rviz(unittest.TestCase):
  def test_create_interactive_6dof(self):
    imarker = cu.rviz.create_interactive_6dof('name')

  def test_create_interactive_mesh(self):
    imarker = cu.rviz.create_interactive_mesh('name', 'resource')

  def test_create_mesh_marker(self):
    marker = cu.rviz.create_mesh_marker(1, 'name', 'resource')

  def test_create_points_marker(self):
    points = [[1,2,3] for _ in range(10)]
    marker = cu.rviz.create_points_marker(1, points)

  def test_create_text_marker(self):
    marker = cu.rviz.create_text_marker(1, 'text')
    marker = cu.rviz.create_text_marker(1, 'text', position=np.random.rand(3))

  def test_get_safe_stamp(self):
    stamp = cu.rviz.get_safe_stamp()
