#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: uri
   :platform: Unix
   :synopsis: Platform dependent rocon_uri methods.

Simple methods that introspect the running platform to generate a rocon uri.
"""

##############################################################################
# Imports
##############################################################################

import rospkg.os_detect
import rocon_python_utils

##############################################################################
# Methods
##############################################################################


def generate_platform_rocon_uri(robot_type, robot_name):
    """
    Detect the platform and rosdistro underneath and determine
    an appropriate rocon uri. It is up to the user to check that this is
    a valid rocon uri for comparison checks (e.g. robot type is officially
    registered).

    :param str robot_type: one of the robot types listed in the rocon uri yaml database
    :param str name: name of the robot
    """
    # This might be naive and only work well on ubuntu...
    os_codename = rospkg.os_detect.OsDetect().get_codename()
    rocon_uri = "rocon:/" + robot_type.lower().replace(' ', '_') + \
                "/" + robot_name.lower().replace(' ', '_') + \
                "/" + rocon_python_utils.ros.get_rosdistro() + \
                "/" + os_codename
    return rocon_uri
