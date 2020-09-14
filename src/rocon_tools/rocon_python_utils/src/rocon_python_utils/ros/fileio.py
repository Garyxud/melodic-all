#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: ros.paths
   :platform: Unix
   :synopsis: Helpers for finding and create cache using rocon.

This module contains helpers that find or creat cache using rocon

----

"""

##############################################################################
# Imports
##############################################################################

# system
import os

# ros
import rospkg

##############################################################################
# Paths
##############################################################################


def get_rocon_home():
    '''
      Retrieve the location of the rocon home directory for using rocon components.
      If it is not existed, create new directory and return this path

      @return the service manager home directory (path object).
      @type str
    '''
    rocon_home = os.path.join(rospkg.get_ros_home(), 'rocon')
    if not os.path.isdir(rocon_home):
        os.makedirs(rocon_home)
    return os.path.join(rospkg.get_ros_home(), 'rocon')


def is_validation_file(file_path):
    '''
    Investigate whether file is validated or not as checking file existed in path and file size

    :param file_path: full file path to check
    :type file_path: string

    :return: validation of file
    :rtype: bool
    '''
    is_validated = True
    if not os.path.isfile(file_path) or os.stat(file_path).st_size <= 0:
        is_validated = False
    return is_validated
