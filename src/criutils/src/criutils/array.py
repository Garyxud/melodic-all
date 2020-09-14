#! /usr/bin/env python
import numpy as np


def unique(data):
  """
  Find the unique elements of an array `row-wise` and returns the sorted unique
  elements of an array.

  Parameters
  ----------
  data: array_like
    Input array.

  Returns
  -------
  res: array_like
    The sorted unique array.
  """
  order = np.lexsort(data.T)
  data = data[order]
  diff = np.diff(data, axis=0)
  ui = np.ones(len(data), 'bool')
  ui[1:] = (diff != 0).any(axis=1)
  return data[ui]
