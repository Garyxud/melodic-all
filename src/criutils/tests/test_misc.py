#! /usr/bin/env python
import unittest
import criutils as cu


class Test_misc(unittest.TestCase):
  def test_has_keys(self):
    # Case when the data is not a dict
    data = ['a']
    keys = ['a']
    res = cu.misc.has_keys(data, keys)
    self.assertFalse(res)
    # Edge case when the passed keys is a string
    data = {'a':1, 'b':2, 'c':3}
    keys = 'abc'
    res = cu.misc.has_keys(data, keys)
    self.assertTrue(res)
    # Typical use case
    data = {'a':1, 'b':2, 'c':3}
    keys = ['a','b']
    res = cu.misc.has_keys(data, keys)
    self.assertTrue(res)
