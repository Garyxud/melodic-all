#!/usr/bin/env python
import rospy
import logging
import rostopic
from .logger import initialize_logging


def read_parameter(name, default):
  """
  Get a parameter from the ROS parameter server. If it's not found, a
  warning is printed.

  Parameters
  ----------
  name: string
    Parameter name
  default: Object
    Default value for the parameter. The type of the default object defines the
    type of the parameter

  Returns
  ----------
  value: Object
    If found, the read parameter. Otherwise, `default` is returned.
  """
  # Check roscore is running
  try:
    rostopic.get_topic_class('/rosout')
    rosmaster_running = True
  except (rostopic.ROSTopicIOException, ValueError):
    rosmaster_running = False
  # Act accordingly
  value = default
  if rosmaster_running:
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' %
                                                              (name, default))
    value = rospy.get_param(name, default)
  else:
    logger = logging.getLogger('read_parameter')
    initialize_logging(format_level=logging.INFO)
    logger.warn('roscore not found, parameter [%s] set to default: %s' %
                                                              (name, default))
  return value
