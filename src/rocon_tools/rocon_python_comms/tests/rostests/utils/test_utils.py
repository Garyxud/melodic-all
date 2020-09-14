#!/usr/bin/env python

""" Testing the find_node function """

##############################################################################
# Imports
##############################################################################

import rocon_console.console as console
import rocon_python_comms
import rospy
import rostest
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs
import unittest


##############################################################################
# Support Methods
##############################################################################

def service_callback(req):
    return std_srvs.EmptyResponse()

def subscriber_callback(msg):
    pass

##############################################################################
# Test Class
##############################################################################

class TestUtils(unittest.TestCase):
    def test_services(self):
        print("")
        print(console.bold + "\n****************************************************************************************" + console.reset)
        print(console.bold + "* Services" + console.reset)
        print(console.bold + "****************************************************************************************" + console.reset)
        print("")
        services = rocon_python_comms.utils.Services(
            [
                ('~dude', std_srvs.Empty, service_callback),
                ('/dude/bob', std_srvs.Empty, service_callback),
            ]
        )
        assert ('dude' in services.__dict__.keys())
        assert ('bob' in services.__dict__.keys())

    def test_service_proxies(self):
        print("")
        print(console.bold + "\n****************************************************************************************" + console.reset)
        print(console.bold + "* Service Proxies" + console.reset)
        print(console.bold + "****************************************************************************************" + console.reset)
        print("")
        service_proxies = rocon_python_comms.utils.ServiceProxies(
            [
                ('~dude', std_srvs.Empty),
                ('/dude/bob', std_srvs.Empty),
            ]
        )
        assert ('dude' in service_proxies.__dict__.keys())
        assert ('bob' in service_proxies.__dict__.keys())

    def test_publishers(self):
        print("")
        print(console.bold + "\n****************************************************************************************" + console.reset)
        print(console.bold + "* Publishers" + console.reset)
        print(console.bold + "****************************************************************************************" + console.reset)
        print("")
        publishers = rocon_python_comms.utils.Publishers(
            [
                ('~foo', std_msgs.String, True, 5),
                ('/foo/bar', std_msgs.String, False, 5),
                ('dude', '/dude/joe', std_msgs.String, False, 5),
            ]
        )
        assert('foo' in publishers.__dict__.keys())
        assert('bar' in publishers.__dict__.keys())
        assert('dude' in publishers.__dict__.keys())

    def test_subscribers(self):
        print("")
        print(console.bold + "\n****************************************************************************************" + console.reset)
        print(console.bold + "* Subscribers" + console.reset)
        print(console.bold + "****************************************************************************************" + console.reset)
        print("")
        subscribers = rocon_python_comms.utils.Subscribers(
            [
                ('~dudette', std_msgs.String, subscriber_callback),
                ('/dudette/jane', std_msgs.String, subscriber_callback),
            ]
        )
        assert('dudette' in subscribers.__dict__.keys())
        assert('jane' in subscribers.__dict__.keys())

if __name__ == '__main__':
    rospy.init_node("test_utils")
    rostest.rosrun('rocon_python_comms',
                   'test_utils',
                   TestUtils)
