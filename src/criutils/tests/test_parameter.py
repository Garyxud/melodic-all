#! /usr/bin/env python
import unittest
import criutils as cu


class Test_parameter(unittest.TestCase):
  def test_read_parameter(self):
    # Try the global case
    value = cu.read_parameter('~test', 100)
    self.assertEqual(value, 100)
    # Try the nested case
    value = cu.parameter.read_parameter('~test', 100)
    self.assertEqual(value, 100)
