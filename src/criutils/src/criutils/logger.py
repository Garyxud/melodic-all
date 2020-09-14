#!/usr/bin/env python

# Copyright (c) 2017, CRI Group, NTU Singapore.
# All rights reserved.
# Authors: Francisco Suarez-Ruiz <fsuarez6@gmail.com>
#
# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Michael Koval <mkoval@cs.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of CRI Group nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import collections, logging, sys, warnings


class ColoredFormatter(logging.Formatter):
  """
  A formatter that allows colors to be placed in the format string.

  Intended to help in creating more readable logging output.
  """
  def __init__(self, formatter):
    """
    Set the format and colors the ColoredFormatter will use.

    Parameters
    ----------
    formatter: logging.Formatter
      Formatter instance used to convert a LogRecord to text.
    """
    self._default_formatter = formatter
    self._color_table = collections.defaultdict(lambda: list())
    self._color_table[logging.CRITICAL] = [ 'red' ]
    self._color_table[logging.ERROR] = [ 'red' ]
    self._color_table[logging.WARNING] = [ 'yellow' ]
    self._color_table[logging.DEBUG] = [ 'green' ]
    # Import termcolor now to fail-fast.
    import termcolor
    self.termcolor = termcolor

  def format(self, record):
    """Customize the message format based on the log level."""
    color_options = self._color_table[record.levelno]
    message = self._default_formatter.format(record)
    return self.termcolor.colored(message, *color_options)


def initialize_logging(spammy_level=logging.WARNING,
                                                format_level=logging.DEBUG):
  """
  Initialize and configure loggers to disable spammy and ROS loggers

  Parameters
  ----------
  spammy_level: int
    Spammy and ROS loggers below this logging level are disabled.
    Default: ``logging.WARNING``
  format_level: int
    Logging level to be use for this logger
    Default: ``logging.DEBUG``

  Returns
  -------
  base_logger: logging.RootLogger
    The base/root logger
  """
  if format_level == logging.DEBUG:
    format_str =  '[%(levelname)s] '
    format_str += '[%(name)s:%(filename)s:%(lineno)d]:%(funcName)s: '
    format_str += '%(message)s'
  elif format_level == logging.INFO:
    format_str = '[%(levelname)s] [%(name)s]: %(message)s'
  else:
    format_str = logging.BASIC_FORMAT
  formatter = logging.Formatter(format_str)
  # Remove all of the existing handlers.
  base_logger = logging.getLogger()
  for handler in list(base_logger.handlers):
    base_logger.removeHandler(handler)
  # Add the custom handler
  handler = logging.StreamHandler(sys.stdout)
  handler.setLevel(logging.DEBUG)
  handler.setFormatter(formatter)
  base_logger.addHandler(handler)
  base_logger.setLevel(logging.INFO)
  # Colorize logging output if the termcolor package is available
  try:
    color_formatter = ColoredFormatter(formatter)
    handler.setFormatter(color_formatter)
  except ImportError:
    logging.warning('Install termcolor to colorize log messages.')
  # Disable spammy and ROS loggers
  spammy_logger_names = [
    'rospy.core',
    'rospy.topics',
    'openravepy.databases',
    'openravepy.inversekinematics',
    'openravepy.databases.inversekinematics',
    'trimesh'
  ]
  for spammy_logger_name in spammy_logger_names:
    spammy_logger = logging.getLogger(spammy_logger_name)
    spammy_logger.setLevel(spammy_level)
  # Hide ikfast warnings
  spammy_logger = logging.getLogger('openravepy.ikfast')
  spammy_logger.setLevel(logging.FATAL)
  # Enable deprecation warnings, which are off by default in Python 2.7
  warnings.simplefilter('default')
  return base_logger

def remove_ros_logger():
  """
  Remove ROS logger
  """
  logger = logging.getLogger()
  new_handlers = []
  for handler in logger.handlers:
    if type(handler).__name__ != 'RosStreamHandler':
      new_handlers.append(handler)
  logger.handlers = new_handlers
