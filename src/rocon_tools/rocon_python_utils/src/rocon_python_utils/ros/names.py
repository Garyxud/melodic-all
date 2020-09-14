
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: ros.name
   :platform: Unix
   :synopsis: Tools for handling name in rocon

This tools 
Tod

----

"""

##############################################################################
# Imports
##############################################################################

# system
import os

# ros
import rospkg
import roslib.names

##############################################################################
# Name
##############################################################################


def get_ros_friendly_name(name):
    '''
    Todo
    Change to snake case string from someone because it is common rule in ros.
    .. code-block:: python

           concert_name = get_ros_friendly_name(" ROCON Concert   ")
           print concert_name

           >>> "rocon_concert"

    : param str name: todo    

    :return file name included extension name
    :type str

    '''
    string_name = name
    if not type(string_name) is str:
        string_name = str(string_name)
    return string_name.strip().lower().replace(" ", "_")


def check_extension_name(file_name, extension_name):
    '''
    Check whether file name include extension name. 
    If it does not include extension name, return the file name added extension name.

    :return file name included extension name
    :type str
    '''

    if extension_name in file_name:
        return file_name
    else:
        return file_name + extension_name