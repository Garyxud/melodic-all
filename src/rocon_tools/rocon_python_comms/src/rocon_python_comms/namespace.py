#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: namespace
   :platform: Unix
   :synopsis: Useful methods relating to ros namespace.


This module contains anything relating to introspection or manipulation
of namespace.

----

"""
##############################################################################
# Imports
##############################################################################

import rospy
import rosgraph
import socket
from rosservice import get_service_type, ROSServiceIOException
from .exceptions import NotFoundException, MultipleFoundException

##############################################################################
# Find namespace
##############################################################################


# for dirname use rosgraph.names.namespace()

def basename(name):
    """
    Generate the basename from a ros name, e.g.

    ...code-block:: python

       > basename("~dude")
       > 'dude'
       > basename("/gang/dude")
       > 'dude'

    :param str name: ros name as input
    :returns: name stripped up until the last slash or tilde character.
    :rtype: str
    """
    return name.rsplit('/', 1)[-1].rsplit('~', 1)[-1]


def find_service_namespace(service_name, service_type=None, unique=False):
    '''
    Find a namespace corresponding with service name and service type.

    :param str service_name: unresolved name of the service looked for (e.g. 'get_interactions', not '/concert/interactions/get_interactions')
    :param str service_type: type name of service looked for (e.g. std_msgs/String) if service_type is None, it does not check type
    :param bool unique: flag to select the lookup behaviour (single/multiple results)

    :returns: the namespace corresponding with service name and type(unique) or list of namespaces (non-unique)
    :rtype: str

    :raises: :exc:`.MultipleFoundException` If it find multiple service if unique flag is on
    :raises: :exc:`.NotFoundException` If it does not find service name in registerd
    :raises: :exc:`.ROSServiceIOException` If it disconnect with ros master
    '''
    try:
        master = rosgraph.Master(rospy.get_name())
        _, _, srvs = master.getSystemState()
        found_namespaces = []
        for s, nodelist in srvs:
            if not service_type:
                if s.split('/')[-1] == service_name and get_service_type(s) == service_type:
                    found_namespaces.append(nodelist[0])
            else:
                if s.split('/')[-1] == service_name:
                    found_namespaces.append(nodelist[0])
        if len(found_namespaces) == 0:
            raise NotFoundException(str(service_name))
        if unique:
            if len(found_namespaces) > 1:
                raise MultipleFoundException(str(found_namespaces))
            else:
                return found_namespaces[0]
        else:
            return found_namespaces
    except socket.error:
        raise ROSServiceIOException("unable to communicate with master!")
