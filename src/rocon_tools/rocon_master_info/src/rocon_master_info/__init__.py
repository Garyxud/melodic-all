#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
This is the top-level namespace of the rocon_master_info_ ROS
package. It provides a ros node for advertising basic information about
a ros master.

.. _rocon_master_info: http://wiki.ros.org/rocon_master_info

"""
##############################################################################
# Imports
##############################################################################

from .master import RoconMaster
from .master_info import main, get_master_info
