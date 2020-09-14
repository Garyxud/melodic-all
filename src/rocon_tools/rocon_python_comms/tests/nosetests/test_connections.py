#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises
import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_console.console as console

import rospy
import rosgraph

##############################################################################
# Tests
##############################################################################

# def test_experiments():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* Experiments" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
#     connection1 = rocon_python_comms.Connection(rocon_std_msgs.Connection.PUBLISHER, "/foo/talker", "/foo")
#     print("Connection: %s" % connection1)
#     print("Msg: %s" % connection1.msg)
#     connection2 = rocon_python_comms.Connection(rocon_std_msgs.Connection.PUBLISHER, "/bar/talker", "/bar")
#     connection3 = rocon_python_comms.Connection(rocon_std_msgs.Connection.PUBLISHER, "/foobar/talker", "/foobar", "std_msgs/String")
#     connections = [connection1, connection2, connection3]
#     connection4 = rocon_python_comms.Connection(rocon_std_msgs.Connection.PUBLISHER, "/bar/talker", "/bar", "std_msgs/String")
#     assert (connection4 in connections)
#     connection_cache = rocon_python_comms.ConnectionCache()
#     connection_cache.update()
#     print("Connections: \n%s" % connection_cache._connections)
#     print(console.cyan + " - %s" % invalid_hardware_platform + console.reset)
#     see rocon_uri nosetests for exception asserts
    
def test_equality():
    print("")
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Operators" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
    connection1 = rocon_python_comms.Connection(rocon_std_msgs.Connection.PUBLISHER, "/foo/talker", "/foo")
    connection2 = rocon_python_comms.Connection(rocon_std_msgs.Connection.PUBLISHER, "/bar/talker", "/bar")
    connection3 = rocon_python_comms.Connection(rocon_std_msgs.Connection.PUBLISHER, "/foobar/talker", "/foobar", "std_msgs/String")
    connections = [connection1, connection2, connection3]
    connection4 = rocon_python_comms.Connection(rocon_std_msgs.Connection.PUBLISHER, "/bar/talker", "/bar", "std_msgs/String")
    assert (connection4 == connection2)
    assert (connection4 != connection3)
    print(console.cyan + "Equality Operators: " + console.yellow + "OK" + console.reset)
    assert (connection4 in connections)
    print(console.cyan + "List Containment: " + console.yellow + "OK" + console.reset)

