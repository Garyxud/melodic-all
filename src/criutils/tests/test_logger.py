#! /usr/bin/env python
import unittest
import logging
import criutils as cu


class Test_logger(unittest.TestCase):
  def test_logger(self):
    # Initialize
    cu.logger.initialize_logging(format_level=logging.DEBUG)
    cu.logger.initialize_logging(format_level=logging.INFO)
    cu.logger.initialize_logging(format_level=logging.WARNING)
    # Remove ROS logger
    cu.logger.remove_ros_logger()
