#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: ros.icons
   :platform: Unix
   :synopsis: Icon manipulation.

This module contains converters and ros resource finders for icons and icon
messages.

----

"""
##############################################################################
# Imports
##############################################################################

# system
import os

# ros
import rocon_std_msgs.msg as rocon_std_msgs

# local
from .resources import find_resource_from_string

##############################################################################
# Functions
##############################################################################


def icon_to_msg(filename):
    '''
      Loads the specified filename and puts in
      `rocon_std_msgs.Icon`_ format

      :param str filename: to the icon resource.
      :returns: the icon in msg format
      :rtype: `rocon_std_msgs.Icon`_

      .. include:: weblinks.rst
    '''
    icon = rocon_std_msgs.Icon()
    if filename is None or not filename:
        return icon
    unused_basename, extension = os.path.splitext(filename)
    if extension.lower() == ".jpg" or extension.lower() == ".jpeg":
        icon.format = "jpeg"
    elif extension.lower() == ".png":
        icon.format = "png"
    else:
        icon.format = ""
        return icon
    icon.data = open(filename, "rb").read()
    return icon


def icon_resource_to_msg(resource):
    '''
      Loads the icon resource and puts in
      `rocon_std_msgs.Icon`_ format

      :param str resource: resource identifier (package/name)

      :returns: the icon in msg format
      :rtype: `rocon_std_msgs.Icon`_

      :raises: :exc:`.rospkg.ResourceNotFound` raised if the resource is not found or has an inappropriate extension.
    '''
    filename = find_resource_from_string(resource)
    icon = icon_to_msg(filename)
    icon.resource_name = resource
    return icon
