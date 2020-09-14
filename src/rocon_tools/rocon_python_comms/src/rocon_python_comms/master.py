#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: master
   :platform: Unix
   :synopsis: Utilities for working with the ros master.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import rospy
import socket

##############################################################################
# Methods
##############################################################################


def check():
    """
    Detect to see if there is a ros master up and running
    """
    # I prefer to check the rosparam server - hitting the master could be
    # costly and we always run rosparam with the master.
    # If instead you want to check the master directly, try something
    # like the master api calls, like getSystemState (althought that is a
    # heavy one) and look for the same socket.error getting raised.
    #
    # Ultimately it would be nice to have direct access to checking the state
    # of rospy.client._init_node_args in many cases as all we need to know
    # is if the node initialised.
    try:
        unused_foo = rospy.get_param('foo', default='bar')
        return True
    except socket.error:
        return False
