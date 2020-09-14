#! /usr/bin/env python
def has_keys(data, keys):
  """
  Check whether a dictionary has all the given keys.

  Parameters
  ----------
  data: dict
    Input dictionary.
  keys: list
    Expected keys to be found in the dictionary

  Returns
  -------
  res: bool
    True if all the keys are found in the dict, false otherwise.
  """
  if not isinstance(data, dict):
    return False
  dict_set = set(data.keys())
  return dict_set.issuperset(keys)
