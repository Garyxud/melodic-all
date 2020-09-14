#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
This is the top-level namespace of the rocon_python_comms_ ROS
package. It provides additional python modules beyond rospy for working with
ros python communications.

.. _rocon_python_comms: http://wiki.ros.org/rocon_python_comms

"""
##############################################################################
# Imports
##############################################################################

from .connections import Connection, ConnectionCache, ConnectionCacheNode, ConnectionCacheProxy
from .connections import SUBSCRIBER, PUBLISHER, SERVICE, ACTION_CLIENT, ACTION_SERVER, connection_types
from .exceptions import *
from . import master
from .nodes import find_node
from .publishers import Publisher
from .service_pair_client import ServicePairClient
from .service_pair_server import ServicePairServer
from .services import find_service, service_is_available
from .subscriber_proxy import SubscriberProxy
from .topics import find_topic
from .wall_rate import WallRate
from .namespace import find_service_namespace, basename
from . import utils
