#!/usr/bin/env python

import multiprocessing
from collections import deque

import os
import sys
import pprint
import subprocess
import unittest
import time
from functools import partial

import rospy
import rostest
import roslaunch
import rosgraph
import rosnode
import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import std_msgs.msg as std_msgs

class timeout(object):
    """
    Small useful timeout class
    """
    def __init__(self, seconds):
        self.seconds = seconds

    def __enter__(self):
        self.die_after = time.time() + self.seconds
        return self

    def __exit__(self, type, value, traceback):
        pass

    @property
    def timed_out(self):
        return time.time() > self.die_after


class TestConnectionCache(unittest.TestCase):
    launch = None

    @classmethod
    def setUpClass(cls):

        # initialize the launch API first
        cls.launch = roslaunch.scriptapi.ROSLaunch()
        cls.launch.start()

        # Note this should be called only once by process.
        # We only run one process here, for all tests
        rospy.init_node('test_connection_cache')

    @classmethod
    def tearDownClass(cls):
        # shutting down process here
        pass

    def setUp(self):
        self.cache = rocon_python_comms.ConnectionCache()
        pass

    def tearDown(self):
        pass

    def chatter_detected(self, topicq_clist, conn_type, node_name):
        """
        detect the chatter publisher in a connection list
        """
        test = False
        for i, conn in enumerate(topicq_clist):  # loop through all connections in the list
            test = (conn.name == '/chatter'
                    and conn.type == conn_type
                    and conn.node.startswith(node_name)  # sometime the node gets suffixes with uuid ??
                    and conn.type_info == 'std_msgs/String'
                    and len(conn.xmlrpc_uri) > 0)
            if test:  # break right away if found
                break
        if not test:
            print "Expected : name:{name} type:{type} node:{node} topic_type:{type_info}".format(name='/chatter', type=conn_type, node=node_name, type_info='std_msgs/String')
            print "NOT FOUND IN LIST : {0}".format(topicq_clist)
        return test

    def string_detected(self, topicq_clist, conn_type, node_name):
        """
        detect the chatter publisher in a connection list
        """
        test = False
        for i, conn in enumerate(topicq_clist):  # loop through all connections in the list
            test = (conn.name == '/test/string'
                    and conn.type == conn_type
                    and conn.node.startswith(node_name)  # sometime the node gets suffixes with uuid ??
                    and conn.type_info == 'std_msgs/String'
                    and len(conn.xmlrpc_uri) > 0)
            if test:  # break right away if found
                break
        if not test:
            print "Expected : name:{name} type:{type} node:{node} topic_type:{type_info}".format(name='/chatter', type=conn_type, node=node_name, type_info='std_msgs/String')
            print "NOT FOUND IN LIST : {0}".format(topicq_clist)
        return test

    def empty_detected(self, topicq_clist, conn_type, node_name):
        """
        detect the chatter publisher in a connection list
        """
        test = False
        for i, conn in enumerate(topicq_clist):  # loop through all connections in the list
            test = (conn.name == '/test/empty'
                    and conn.type == conn_type
                    and conn.node.startswith(node_name)  # sometime the node gets suffixes with uuid ??
                    and conn.type_info == 'std_msgs/Empty'
                    and len(conn.xmlrpc_uri) > 0)
            if test:  # break right away if found
                break
        if not test:
            print "Expected : name:{name} type:{type} node:{node} topic_type:{type_info}".format(name='/chatter', type=conn_type, node=node_name, type_info='std_msgs/String')
            print "NOT FOUND IN LIST : {0}".format(topicq_clist)
        return test

    def add_two_ints_detected(self, svcq_clist, conn_type, node_name):
        """
        detect the add_two_ints service in a connection list
        """
        test = False
        for i, conn in enumerate(svcq_clist):  # lop through all connections in the list
            test = (conn.name == '/add_two_ints'
                    and conn.type == conn_type
                    and conn.node.startswith(node_name)  # sometime the node gets suffixes with uuid ??
                    and len(conn.type_info) > 0
                    and len(conn.xmlrpc_uri) > 0)
            if test:  # break right away if found
                break
        if not test:
            print "Expected : name:{name} type:{type} node:{node}".format(name='/add_two_ints', type=conn_type, node=node_name)
            print "NOT FOUND IN LIST : {0}".format(svcq_clist)
        return test

    def equalMasterSystemState(self, proxySS):
        masterSS = self._master.getSystemState()[2]

        same = True
        # masterSS set included in proxySS set
        for idx, t in enumerate(masterSS):  # connection type
            print "MASTER SYSTEM STATE CONNECTION TYPE {0}".format(t)
            print " -> PROXY SYSTEM STATE CONNECTION TYPE {0}".format(proxySS[idx])
            proxySS_names = [c[0] for c in proxySS[idx]]
            for c in t:  # connection list [name, [nodes]]
                print "MASTER SYSTEM STATE CONNECTION {0}".format(c)
                print " -> CHECK {0} in PROXY NAMES {1}".format(c[0], proxySS_names)
                same = same and c[0] in proxySS_names
                if not same:
                    print("ERROR : {0} not in {1}".format(c[0], proxySS_names))
                    break
                proxySS_conn_nodes = [fpc for pc in proxySS[idx] if c[0] == pc[0] for fpc in pc[1]]
                for n in c[1]:
                    print " -> CHECK {0} in PROXY NODES {1}".format(n, proxySS_conn_nodes)
                    same = same and n in proxySS_conn_nodes
                    if not same:
                        print("ERROR : {0} not in {1}".format(n, proxySS_conn_nodes))
                        break

        # proxySS set included in masterSS set
        for idx, t in enumerate(proxySS):  # connection type
            print "PROXY SYSTEM STATE CONNECTION TYPE {0}".format(t)
            print " -> MASTER SYSTEM STATE CONNECTION TYPE {0}".format(masterSS[idx])
            masterSS_names = [c[0] for c in masterSS[idx]]
            for c in t:  # connection list [name, [nodes]]
                print "PROXY SYSTEM STATE CONNECTION {0}".format(c)
                print " -> CHECK {0} in MASTER NAMES {1}".format(c[0], masterSS_names)
                same = same and c[0] in masterSS_names
                if not same:
                    print("ERROR : {0} not in {1}".format(c[0], masterSS_names))
                    break
                masterSS_conn_nodes = [fmc for mc in masterSS[idx] if c[0] == mc[0] for fmc in mc[1]]
                for n in c[1]:
                    print " -> CHECK {0} in MASTER NODES {1}".format(n, masterSS_conn_nodes)
                    same = same and n in masterSS_conn_nodes
                    if not same:
                        print("ERROR : {0} not in {1}".format(n, masterSS_conn_nodes))
                        break
        return same

    def equalMasterTopicTypes(self, proxyTT):
        masterTT = self._master.getTopicTypes()[2]

        same = True
        # proxyTT set included in masterTT set
        for pt in proxyTT:  # [topic_name, topic_type]
            print "PROXY SYSTEM STATE TOPIC {0}".format(pt)
            mtl = [mt for mt in masterTT if mt[0] == pt[0]]
            print " -> MASTER SYSTEM STATE TOPIC {0}".format(mtl)
            same = same and len(mtl) == 1
            if not same:
                print("ERROR : {1} has more or less than 1 element".format(pt, mtl))
                break
            same = same and pt[1] == mtl[0][1]
            if not same:
                print("ERROR : {0} not in {1}".format(pt, mtl))
                break

        if same and False:  # currently disabled since it seems master API doesnt removed topics which do not have any pub / sub anymore

            # masterTT set included in proxyTT set
            for mt in masterTT:  # [topic_name, topic_type]
                print "MASTER SYSTEM STATE TOPIC {0}".format(mt)
                ptl = [pt for pt in proxyTT if pt[0] == mt[0]]
                print " -> PROXY SYSTEM STATE TOPIC {0}".format(ptl)
                same = same and len(ptl) == 1
                if not same:
                    print("ERROR : {1} has more or less than 1 element".format(mt, ptl))
                    break
                same = same and mt[1] == ptl[0][1]
                if not same:
                    print("ERROR : {0} not in {1}".format(mt, ptl))
                    break

        return same

    def test_detect_publisher_added_lost(self):
        # Start a dummy node
        talker_node = roslaunch.core.Node('roscpp_tutorials', 'talker')
        process = self.launch.launch(talker_node)
        try:

            # Loop a bit so we can detect the publisher appeared
            with timeout(5) as t:
                found_in_new = False
                while not t.timed_out:

                    new, lost = self.cache.update()
                    # asserting update detected it
                    found_in_new = self.chatter_detected(new[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/talker')
                    if found_in_new:
                        break
                    time.sleep(0.2)

                assert found_in_new
            # asserting it s been added to the internal list of connections in the cache
            assert self.chatter_detected(self.cache.connections[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/talker')

            # Start a dummy node to trigger a change
            server_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
            distraction_process = self.launch.launch(server_node)
            try:

                # Loop a bit so we can detect the distraction service appeared
                with timeout(5) as t:
                    found_distraction_in_new = False
                    while not t.timed_out:

                        new, lost = self.cache.update()
                        # asserting update detected it
                        found_distraction_in_new = self.add_two_ints_detected(new[rocon_python_comms.SERVICE], rocon_python_comms.SERVICE, '/add_two_ints_server')
                        if found_distraction_in_new:
                            break
                        time.sleep(0.2)

                    assert found_distraction_in_new
                # asserting it s still in the internal list of connections in the cache
                assert self.chatter_detected(self.cache.connections[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/talker')

            finally:
                distraction_process.stop()

        finally:
            process.stop()

        # Loop a bit so we can detect the service is gone
        with timeout(5) as t:
            found_in_lost = False
            while not t.timed_out:

                new, lost = self.cache.update()
                # asserting update detected it
                found_in_lost = self.chatter_detected(lost[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/talker')
                if found_in_lost:
                    break
                time.sleep(0.2)

            assert found_in_lost
        # asserting it s been removed from the internal list of connections in the cache
        assert not self.chatter_detected(self.cache.connections[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/talker')

    def test_detect_publisher_added_lost_unregister(self):
        # Start a dummy publisher
        string_pub = rospy.Publisher('/test/string', std_msgs.String, queue_size=1)
        try:

            # Loop a bit so we can detect the publisher appeared
            with timeout(5) as t:
                found_in_new = False
                while not t.timed_out:

                    new, lost = self.cache.update()
                    # asserting update detected it
                    found_in_new = self.string_detected(new[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/test_connection_cache')
                    if found_in_new:
                        break
                    time.sleep(0.2)

                assert found_in_new
            # asserting it s been added to the internal list of connections in the cache
            assert self.string_detected(self.cache.connections[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/test_connection_cache')

            # Start a dummy publisher to trigger a change
            empty_pub = rospy.Publisher('/test/empty', std_msgs.Empty, queue_size=1)
            try:

                # Loop a bit so we can detect the distraction service appeared
                with timeout(5) as t:
                    found_distraction_in_new = False
                    while not t.timed_out:

                        new, lost = self.cache.update()
                        # asserting update detected it
                        found_distraction_in_new = self.empty_detected(new[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/test_connection_cache')
                        if found_distraction_in_new:
                            break
                        time.sleep(0.2)

                    assert found_distraction_in_new
                # asserting it s still in the internal list of connections in the cache
                assert self.string_detected(self.cache.connections[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/test_connection_cache')

            finally:
                empty_pub.unregister()

        finally:
            string_pub.unregister()

        # Loop a bit so we can detect the service is gone
        with timeout(5) as t:
            found_in_lost = False
            while not t.timed_out:

                new, lost = self.cache.update()
                # asserting update detected it
                found_in_lost = self.string_detected(lost[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/test_connection_cache')
                if found_in_lost:
                    break
                time.sleep(0.2)

            assert found_in_lost
        # asserting it s been removed from the internal list of connections in the cache
        assert not self.string_detected(self.cache.connections[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/test_connection_cache')

    def test_detect_subscriber_added_lost(self):
        # Start a dummy node
        talker_node = roslaunch.core.Node('roscpp_tutorials', 'listener')
        process = self.launch.launch(talker_node)
        try:

            # Loop a bit so we can detect the publisher appeared
            with timeout(5) as t:
                found_in_new = False
                while not t.timed_out:

                    new, lost = self.cache.update()
                    # asserting update detected it
                    found_in_new = self.chatter_detected(new[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/listener')
                    if found_in_new:
                        break
                    time.sleep(0.2)

                assert found_in_new
            # asserting it s been added to the internal list of connections in the cache
            assert self.chatter_detected(self.cache.connections[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/listener')

            # Start a dummy node to trigger a change
            server_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
            distraction_process = self.launch.launch(server_node)
            try:

                # Loop a bit so we can detect the distraction service appeared
                with timeout(5) as t:
                    found_distraction_in_new = False
                    while not t.timed_out:

                        new, lost = self.cache.update()
                        # asserting update detected it
                        found_distraction_in_new = self.add_two_ints_detected(new[rocon_python_comms.SERVICE], rocon_python_comms.SERVICE, '/add_two_ints_server')
                        if found_distraction_in_new:
                            break
                        time.sleep(0.2)

                    assert found_distraction_in_new
                # asserting it s still in the internal list of connections in the cache
                assert self.chatter_detected(self.cache.connections[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/listener')

            finally:
                distraction_process.stop()

        finally:
            process.stop()

        # Loop a bit so we can detect the service is gone
        with timeout(5) as t:
            found_in_lost = False
            while not t.timed_out:

                new, lost = self.cache.update()
                # asserting update detected it
                found_in_lost = self.chatter_detected(lost[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/listener')
                if found_in_lost:
                    break
                time.sleep(0.2)

            assert found_in_lost
        # asserting it s been removed from the internal list of connections in the cache
        assert not self.chatter_detected(self.cache.connections[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/listener')

    def test_detect_subscriber_added_lost_unregister(self):
        def dummy_cb(data):
            pass

        # Start a dummy subscriber
        string_sub = rospy.Subscriber('/test/string', std_msgs.String, dummy_cb)
        try:

            # Loop a bit so we can detect the publisher appeared
            with timeout(5) as t:
                found_in_new = False
                while not t.timed_out:

                    new, lost = self.cache.update()
                    # asserting update detected it
                    found_in_new = self.string_detected(new[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/test_connection_cache')
                    if found_in_new:
                        break
                    time.sleep(0.2)

                assert found_in_new
            # asserting it s been added to the internal list of connections in the cache
            assert self.string_detected(self.cache.connections[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/test_connection_cache')

            # Start a dummy subscriber to trigger a change
            empty_sub = rospy.Subscriber('/test/empty', std_msgs.Empty, dummy_cb)
            try:

                # Loop a bit so we can detect the distraction service appeared
                with timeout(5) as t:
                    found_distraction_in_new = False
                    while not t.timed_out:

                        new, lost = self.cache.update()
                        # asserting update detected it
                        found_distraction_in_new = self.empty_detected(new[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/test_connection_cache')
                        if found_distraction_in_new:
                            break
                        time.sleep(0.2)

                    assert found_distraction_in_new
                # asserting it s still in the internal list of connections in the cache
                assert self.string_detected(self.cache.connections[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/test_connection_cache')

            finally:
                empty_sub.unregister()

        finally:
            string_sub.unregister()

        # Loop a bit so we can detect the service is gone
        with timeout(5) as t:
            found_in_lost = False
            while not t.timed_out:

                new, lost = self.cache.update()
                # asserting update detected it
                found_in_lost = self.string_detected(lost[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/test_connection_cache')
                if found_in_lost:
                    break
                time.sleep(0.2)

            assert found_in_lost
        # asserting it s been removed from the internal list of connections in the cache
        assert not self.string_detected(self.cache.connections[rocon_python_comms.SUBSCRIBER], rocon_python_comms.SUBSCRIBER, '/test_connection_cache')

    def test_detect_service_added_lost(self):
        # Start a dummy node
        talker_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
        process = self.launch.launch(talker_node)
        try:

            # Loop a bit so we can detect the publisher appeared
            with timeout(5) as t:
                found_in_new = False
                while not t.timed_out:

                    new, lost = self.cache.update()
                    # asserting update detected it
                    found_in_new = self.add_two_ints_detected(new[rocon_python_comms.SERVICE], rocon_python_comms.SERVICE, '/add_two_ints_server')
                    if found_in_new:
                        break
                    time.sleep(0.2)

                assert found_in_new
            # asserting it s been added to the internal list of connections in the cache
            assert self.add_two_ints_detected(self.cache.connections[rocon_python_comms.SERVICE], rocon_python_comms.SERVICE, '/add_two_ints_server')

            # Start a dummy node to trigger a change
            server_node = roslaunch.core.Node('roscpp_tutorials', 'talker')
            distraction_process = self.launch.launch(server_node)
            try:

                # Loop a bit so we can detect the distraction service appeared
                with timeout(5) as t:
                    found_distraction_in_new = False
                    while not t.timed_out:

                        new, lost = self.cache.update()
                        # asserting update detected it
                        found_distraction_in_new = self.chatter_detected(new[rocon_python_comms.PUBLISHER], rocon_python_comms.PUBLISHER, '/talker')
                        if found_distraction_in_new:
                            break
                        time.sleep(0.2)

                    assert found_distraction_in_new
                # asserting it s still in the internal list of connections in the cache
                assert self.add_two_ints_detected(self.cache.connections[rocon_python_comms.SERVICE], rocon_python_comms.SERVICE, '/add_two_ints_server')

            finally:
                distraction_process.stop()

        finally:
            process.stop()

        # Loop a bit so we can detect the service is gone
        with timeout(5) as t:
            found_in_lost = False
            while not t.timed_out:

                new, lost = self.cache.update()
                # asserting update detected it
                found_in_lost = self.add_two_ints_detected(lost[rocon_python_comms.SERVICE], rocon_python_comms.SERVICE, '/add_two_ints_server')
                if found_in_lost:
                    break
                time.sleep(0.2)

            assert found_in_lost
        # asserting it s been removed from the internal list of connections in the cache
        assert not self.add_two_ints_detected(self.cache.connections[rocon_python_comms.SERVICE], rocon_python_comms.SERVICE, '/add_two_ints_server')

if __name__ == '__main__':

    # setup_module()
    rostest.rosrun('rocon_python_comms',
                   'test_connection_cache',
                   TestConnectionCache)
    # teardown_module()
