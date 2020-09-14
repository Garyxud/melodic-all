#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
This is the top-level namespace of the rocon_interactions_ ROS
package. It provides python utilities for working with the console.

.. _rocon_interactions: http://wiki.ros.org/rocon_interactions

"""
##############################################################################
# Imports
##############################################################################

from .manager import InteractionsManager
from .loader import InteractionsLoader
from .interactions_table import InteractionsTable
from .pairings import Pairing
from .pairings_table import PairingsTable
from .interactions import Interaction
from exceptions import *
from . import utils

import web_interactions
