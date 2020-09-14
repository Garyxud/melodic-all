#!/usr/bin/env python

import multiprocessing
import threading
from collections import deque

import os
import sys
import pprint
import subprocess
import unittest
import time
from functools import partial
import nose

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


class TestConnectionCacheNode(unittest.TestCase):
    launch = None

    @classmethod
    def setUpClass(cls):

        # initialize the launch API first
        cls.launch = roslaunch.scriptapi.ROSLaunch()
        cls.launch.start()

        # Note this should be called only once by process.
        # We only run one process here, for all tests
        rospy.init_node('test_connection_cache_node')

    @classmethod
    def tearDownClass(cls):
        # shutting down process here
        cls.launch.stop()

    def _list_cb(self, data):
        self.conn_list_msgq.append(data)
        pass

    def _diff_cb(self, data):
        self.conn_diff_msgq.append(data)
        pass

    def _spin_cb(self, data):
        self.spin_freq = data.spin_freq
        self.spin_timer = data.spin_timer

    def user_cb(self, current, added, lost):
        if not hasattr(self, '_ss_lock'):
            self._ss_lock = threading.Lock()
        with self._ss_lock:
            self._current_ss = current
            self._added_ss = added
            self._lost_ss = lost

    def cleanup_user_cb_ss(self):
        if not hasattr(self, '_ss_lock'):
            self._ss_lock = threading.Lock()
        with self._ss_lock:
            self._current_ss = None
            self._added_ss = None
            self._lost_ss = None

    def setUp(self, cacheproxy=None):
        cacheproxy = cacheproxy or partial(rocon_python_comms.ConnectionCacheProxy, user_callback=self.user_cb, diff_opt=False)
        # We prepare our data structure for checking messages
        self.conn_list_msgq = deque()
        self.conn_diff_msgq = deque()
        self.spin_freq = 0.0
        self.cleanup_user_cb_ss()

        # Then we hookup to its topics and prepare a service proxy
        self.conn_list = rospy.Subscriber('/connection_cache/list', rocon_std_msgs.ConnectionsList, self._list_cb)
        self.conn_diff = rospy.Subscriber('/connection_cache/diff', rocon_std_msgs.ConnectionsDiff, self._diff_cb)
        self.set_spin = rospy.Publisher('/connection_cache/spin', rocon_std_msgs.ConnectionCacheSpin, queue_size=1)
        self.get_spin = rospy.Subscriber('/connection_cache/spin', rocon_std_msgs.ConnectionCacheSpin, self._spin_cb)

        # connecting to the master via proxy object
        self._master = rospy.get_master()

        self.node_default_spin_freq = 1  # change this to test different spin speed for the connection cache node
        rospy.set_param("/connection_cache/spin_freq", self.node_default_spin_freq)
        cache_node = roslaunch.core.Node('rocon_python_comms', 'connection_cache.py', name='connection_cache')
        self.cache_process = self.launch.launch(cache_node)

        node_api = None
        with timeout(5) as t:
            while not t.timed_out and node_api is None:
                node_api = rosnode.get_api_uri(self._master, 'connection_cache')

        assert node_api is not None  # make sure the connection cache node is started before moving on.

        # This will block until the cache node is available and has sent latched system state
        self.proxy = cacheproxy(
                list_sub='/connection_cache/list',
                diff_sub='/connection_cache/diff'
        )

        # Making sure System state is set after proxy initialized
        assert self.proxy._system_state is not None

        assert not t.timed_out

    def tearDown(self):
        if self.cache_process:
            self.cache_process.stop()
            while self.cache_process.is_alive():
                time.sleep(0.2)  # waiting for cache node to die
            assert not self.cache_process.is_alive()
            time.sleep(1)  # TODO : investigate : we shouldnt need this
        pass

    def chatter_detected(self, topicq_clist, conn_type, node_name):
        """
        detect the chatter publisher in a connection list
        """
        test = False
        for i, conn in enumerate(topicq_clist):  # lop through all connections in the list
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

    def chatter_chan_detected(self, topicq_cdict, node_name):
        """
        detect the chatter publisher in a connection list
        """
        test = '/chatter' in topicq_cdict.keys() and (
            topicq_cdict['/chatter'].name == '/chatter'
            and topicq_cdict['/chatter'].type == 'std_msgs/String'
            and len([n for n in topicq_cdict['/chatter'].nodes if n[0].startswith(node_name)]) > 0  # sometime the node gets suffixes with uuid
        )
        if not test:
            print "Expected : name:{name} type:{type} node:{node}".format(name='/chatter', node=node_name, type='std_msgs/String')
            print "NOT FOUND IN DICT : {0}".format(topicq_cdict)
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

    def string_chan_detected(self, topicq_cdict, node_name):
        """
        detect the chatter publisher in a connection list
        """
        test = '/test/string' in topicq_cdict.keys() and (
            topicq_cdict['/test/string'].name == '/test/string'
            and topicq_cdict['/test/string'].type == 'std_msgs/String'
            and len([n for n in topicq_cdict['/test/string'].nodes if n[0].startswith(node_name)]) > 0  # sometime the node gets suffixes with uuid
        )
        if not test:
            print "Expected : name:{name} type:{type} node:{node}".format(name='/test/string', node=node_name, type='std_msgs/String')
            print "NOT FOUND IN DICT : {0}".format(topicq_cdict)
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

    def add_two_ints_chan_detected(self, svcq_cdict, node_name):
        """
        detect the chatter publisher in a connection list
        """
        test = '/add_two_ints' in svcq_cdict.keys() and (
            svcq_cdict['/add_two_ints'].name == '/add_two_ints'
            and svcq_cdict['/add_two_ints'].type is not None
            and svcq_cdict['/add_two_ints'].xmlrpc_uri is not None
            and len([n for n in svcq_cdict['/add_two_ints'].nodes if n[0].startswith(node_name)]) > 0  # sometime the node gets suffixes with uuid
        )
        if not test:
            print "Expected : name:{name} node:{node}".format(name='/chatter', node=node_name)
            print "NOT FOUND IN DICT : {0}".format(svcq_cdict)
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

    def test_detect_publisher_added_lost(self,):
        # Start a dummy node
        talker_node = roslaunch.core.Node('roscpp_tutorials', 'talker')
        process = self.launch.launch(talker_node)
        try:
            added_publisher_detected = {'list': False, 'diff': False, 'cb_list': False, 'cb_diff': False}

            # Loop a bit so we can detect the topic
            with timeout(5/self.node_default_spin_freq) as t:
                while not t.timed_out:
                    # Here we only check the last message received
                    if not added_publisher_detected['list'] and self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.PUBLISHER, '/talker'):  # if we find it
                        added_publisher_detected['list'] = True

                    if not added_publisher_detected['diff'] and self.conn_diff_msgq and self.chatter_detected(self.conn_diff_msgq[-1].added, rocon_python_comms.PUBLISHER, '/talker'):  # if we find it
                        added_publisher_detected['diff'] = True

                    # asserting in proxy callback as well
                    with self._ss_lock:
                        if not added_publisher_detected['cb_list'] and self._current_ss and self.chatter_chan_detected(self._current_ss.publishers, '/talker'):
                            added_publisher_detected['cb_list'] = True
                        if self.proxy.diff_opt and self._added_ss and self.chatter_chan_detected(self._added_ss.publishers, '/talker'):
                            added_publisher_detected['cb_diff'] = True

                    if added_publisher_detected['list'] and added_publisher_detected['diff'] and added_publisher_detected['cb_list'] and (added_publisher_detected['cb_diff'] or not self.proxy.diff_opt):
                        break
                    time.sleep(0.2)

            assert added_publisher_detected['list']
            assert added_publisher_detected['diff']
            assert added_publisher_detected['cb_list']
            assert (self.proxy.diff_opt and added_publisher_detected['cb_diff']) or (not self.proxy.diff_opt and not added_publisher_detected['cb_diff'])

            time.sleep(0.2)
            # asserting in proxy as well
            assert self.equalMasterSystemState(self.proxy.getSystemState())
            assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

            # Start a dummy node to trigger a change
            server_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
            distraction_process = self.launch.launch(server_node)
            try:
                still_publisher_detected = {'list': False, 'cb_list': False}

                # Loop a bit so we can detect the topic
                with timeout(5/self.node_default_spin_freq) as t:
                    while not t.timed_out:
                        # Here we only check the last message received
                        if not still_publisher_detected['list'] and self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.PUBLISHER, '/talker'):  # if we find it
                            still_publisher_detected['list'] = True

                        # asserting in proxy callback as well
                        with self._ss_lock:
                            if not still_publisher_detected['cb_list'] and self._current_ss and self.chatter_chan_detected(self._current_ss.publishers, '/talker'):
                                still_publisher_detected['cb_list'] = True

                        if still_publisher_detected['list'] and still_publisher_detected['cb_list']:
                            break
                        time.sleep(0.2)

                assert still_publisher_detected['list']
                assert still_publisher_detected['cb_list']
                time.sleep(0.2)
                # asserting in proxy as well
                assert self.equalMasterSystemState(self.proxy.getSystemState())
                assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())
            finally:
                distraction_process.stop()

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

        finally:
            process.stop()

        lost_publisher_detected = {'list': False, 'diff': False, 'cb_list': False, 'cb_diff': False}

        # Loop a bit so we can detect the topic is gone
        with timeout(5/self.node_default_spin_freq) as t:
            while not t.timed_out:
                # Here we only check the last message received
                if not lost_publisher_detected['list'] and self.conn_list_msgq and not self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.PUBLISHER, '/talker'):  # if we DONT find it
                    lost_publisher_detected['list'] = True

                if not lost_publisher_detected['diff'] and self.conn_diff_msgq and self.chatter_detected(self.conn_diff_msgq[-1].lost, rocon_python_comms.PUBLISHER, '/talker'):  # if we find it
                    lost_publisher_detected['diff'] = True

                # asserting in proxy callback as well
                with self._ss_lock:
                    if not lost_publisher_detected['cb_list'] and self._current_ss and not self.chatter_chan_detected(self._current_ss.publishers, '/talker'):
                        lost_publisher_detected['cb_list'] = True
                    if not lost_publisher_detected['cb_diff'] and self._lost_ss and self.chatter_chan_detected(self._lost_ss.publishers, '/talker'):
                        lost_publisher_detected['cb_diff'] = True

                if lost_publisher_detected['list'] and lost_publisher_detected['diff'] and lost_publisher_detected['cb_list'] and (lost_publisher_detected['cb_diff'] or not self.proxy.diff_opt):
                    break
                time.sleep(0.2)

        assert lost_publisher_detected['list']
        assert lost_publisher_detected['diff']
        assert lost_publisher_detected['cb_list']
        assert (self.proxy.diff_opt and lost_publisher_detected['cb_diff']) or (not self.proxy.diff_opt and not lost_publisher_detected['cb_diff'])
        time.sleep(0.2)
        # asserting in proxy as well
        assert self.equalMasterSystemState(self.proxy.getSystemState())
        assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())

    def test_detect_publisher_added_lost_unregister(self):
        # Start a dummy publisher
        string_pub = rospy.Publisher('/test/string', std_msgs.String, queue_size=1)
        try:
            added_publisher_detected = {'list': False, 'diff': False, 'cb_list': False, 'cb_diff': False}

            # Loop a bit so we can detect the topic
            with timeout(5/self.node_default_spin_freq) as t:
                while not t.timed_out:
                    # Here we only check the last message received
                    if not added_publisher_detected['list'] and self.conn_list_msgq and self.string_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.PUBLISHER, '/test_connection_cache_node'):  # if we find it
                        added_publisher_detected['list'] = True

                    if not added_publisher_detected['diff'] and self.conn_diff_msgq and self.string_detected(self.conn_diff_msgq[-1].added, rocon_python_comms.PUBLISHER, '/test_connection_cache_node'):  # if we find it
                        added_publisher_detected['diff'] = True

                    # asserting in proxy callback as well
                    with self._ss_lock:
                        if not added_publisher_detected['cb_list'] and self._current_ss and self.string_chan_detected(self._current_ss.publishers, '/test_connection_cache_node'):
                            added_publisher_detected['cb_list'] = True
                        if self.proxy.diff_opt and self._added_ss and self.string_chan_detected(self._added_ss.publishers, '/test_connection_cache_node'):
                            added_publisher_detected['cb_diff'] = True

                    if added_publisher_detected['list'] and added_publisher_detected['diff'] and added_publisher_detected['cb_list'] and (added_publisher_detected['cb_diff'] or not self.proxy.diff_opt):
                        break
                    time.sleep(0.2)

            assert added_publisher_detected['list']
            assert added_publisher_detected['diff']
            assert added_publisher_detected['cb_list']
            assert (self.proxy.diff_opt and added_publisher_detected['cb_diff']) or (not self.proxy.diff_opt and not added_publisher_detected['cb_diff'])

            time.sleep(0.2)
            # asserting in proxy as well
            assert self.equalMasterSystemState(self.proxy.getSystemState())
            assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

            # Start a dummy extra publisher to trigger a change
            empty_pub = rospy.Publisher('/test/empty', std_msgs.String, queue_size=1)
            try:
                still_publisher_detected = {'list': False, 'cb_list': False}

                # Loop a bit so we can detect the topic
                with timeout(5/self.node_default_spin_freq) as t:
                    while not t.timed_out:
                        # Here we only check the last message received
                        if not still_publisher_detected['list'] and self.conn_list_msgq and self.string_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.PUBLISHER, '/test_connection_cache_node'):  # if we find it
                            still_publisher_detected['list'] = True

                        # asserting in proxy callback as well
                        with self._ss_lock:
                            if not still_publisher_detected['cb_list'] and self._current_ss and self.string_chan_detected(self._current_ss.publishers, '/test_connection_cache_node'):
                                still_publisher_detected['cb_list'] = True

                        if still_publisher_detected['list'] and still_publisher_detected['cb_list']:
                            break
                        time.sleep(0.2)

                assert still_publisher_detected['list']
                assert still_publisher_detected['cb_list']
                time.sleep(0.2)
                # asserting in proxy as well
                assert self.equalMasterSystemState(self.proxy.getSystemState())
                assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())
            finally:
                empty_pub.unregister()

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

        finally:
            string_pub.unregister()

        lost_publisher_detected = {'list': False, 'diff': False, 'cb_list': False, 'cb_diff': False}

        # Loop a bit so we can detect the topic is gone
        with timeout(5/self.node_default_spin_freq) as t:
            while not t.timed_out:
                # Here we only check the last message received
                if not lost_publisher_detected['list'] and self.conn_list_msgq and not self.string_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.PUBLISHER, '/test_connection_cache_node'):  # if we DONT find it
                    lost_publisher_detected['list'] = True

                if not lost_publisher_detected['diff'] and self.conn_diff_msgq and self.string_detected(self.conn_diff_msgq[-1].lost, rocon_python_comms.PUBLISHER, '/test_connection_cache_node'):  # if we find it
                    lost_publisher_detected['diff'] = True

                # asserting in proxy callback as well
                with self._ss_lock:
                    if not lost_publisher_detected['cb_list'] and self._current_ss and not self.string_chan_detected(self._current_ss.publishers, '/test_connection_cache_node'):
                        lost_publisher_detected['cb_list'] = True
                    if not lost_publisher_detected['cb_diff'] and self._lost_ss and self.string_chan_detected(self._lost_ss.publishers, '/test_connection_cache_node'):
                        lost_publisher_detected['cb_diff'] = True

                if lost_publisher_detected['list'] and lost_publisher_detected['diff'] and lost_publisher_detected['cb_list'] and (lost_publisher_detected['cb_diff'] or not self.proxy.diff_opt):
                    break
                time.sleep(0.2)

        assert lost_publisher_detected['list']
        assert lost_publisher_detected['diff']
        assert lost_publisher_detected['cb_list']
        assert (self.proxy.diff_opt and lost_publisher_detected['cb_diff']) or (not self.proxy.diff_opt and not lost_publisher_detected['cb_diff'])
        time.sleep(0.2)
        # asserting in proxy as well
        assert self.equalMasterSystemState(self.proxy.getSystemState())
        assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())

    def test_detect_subscriber_added_lost(self):
        # Start a dummy node
        listener_node = roslaunch.core.Node('roscpp_tutorials', 'listener')
        process = self.launch.launch(listener_node)
        try:
            added_subscriber_detected = {'list': False, 'diff': False, 'cb_list': False, 'cb_diff': False}

            # Loop a bit so we can detect the topic
            with timeout(5/self.node_default_spin_freq) as t:
                while not t.timed_out:
                    # Here we only check the last message received
                    if not added_subscriber_detected['list'] and self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                        added_subscriber_detected['list'] = True

                    if not added_subscriber_detected['diff'] and self.conn_diff_msgq and self.chatter_detected(self.conn_diff_msgq[-1].added, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                        added_subscriber_detected['diff'] = True

                    # asserting in proxy callback as well
                    with self._ss_lock:
                        if not added_subscriber_detected['cb_list'] and self._current_ss and self.chatter_chan_detected(self._current_ss.subscribers, '/listener'):
                            added_subscriber_detected['cb_list'] = True
                        if not added_subscriber_detected['cb_diff'] and self._added_ss and self.chatter_chan_detected(self._added_ss.subscribers, '/listener'):
                            added_subscriber_detected['cb_diff'] = True

                    if added_subscriber_detected['list'] and added_subscriber_detected['diff'] and added_subscriber_detected['cb_list'] and (added_subscriber_detected['cb_diff'] or not self.proxy.diff_opt):
                        break
                    time.sleep(0.2)

            assert added_subscriber_detected['list']
            assert added_subscriber_detected['diff']
            assert added_subscriber_detected['cb_list']
            assert (self.proxy.diff_opt and added_subscriber_detected['cb_diff']) or (not self.proxy.diff_opt and not added_subscriber_detected['cb_diff'])

            # asserting in proxy as well
            time.sleep(0.2)
            assert self.equalMasterSystemState(self.proxy.getSystemState())
            assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

            # Start a dummy node to trigger a change
            server_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
            distraction_process = self.launch.launch(server_node)
            try:
                still_subscriber_detected = {'list': False, 'cb_list': False}

                # Loop a bit so we can detect the topic
                with timeout(5/self.node_default_spin_freq) as t:
                    while not t.timed_out:
                        # Here we only check the last message received
                        if not still_subscriber_detected['list'] and self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                            still_subscriber_detected['list'] = True

                        # asserting in proxy callback as well
                        with self._ss_lock:
                            if not still_subscriber_detected['cb_list'] and self._current_ss and self.chatter_chan_detected(self._current_ss.subscribers, '/listener'):
                                still_subscriber_detected['cb_list'] = True

                        if still_subscriber_detected['list'] and still_subscriber_detected['cb_list']:
                            break
                        time.sleep(0.2)

                assert still_subscriber_detected['list']
                assert still_subscriber_detected['cb_list']
                time.sleep(0.2)
                # asserting in proxy as well
                assert self.equalMasterSystemState(self.proxy.getSystemState())
                assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())
            finally:
                distraction_process.stop()

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

        finally:
            process.stop()

        lost_subscriber_detected = {'list': False, 'diff': False, 'cb_list': False, 'cb_diff': False}

        # Loop a bit so we can detect the topic is gone
        with timeout(5/self.node_default_spin_freq) as t:
            while not t.timed_out:
                # Here we only check the last message received
                if not lost_subscriber_detected['list'] and self.conn_list_msgq and not self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we DONT find it
                    lost_subscriber_detected['list'] = True

                if not lost_subscriber_detected['diff'] and self.conn_diff_msgq and self.chatter_detected(self.conn_diff_msgq[-1].lost, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                    lost_subscriber_detected['diff'] = True

                # asserting in proxy callback as well
                with self._ss_lock:
                    if not lost_subscriber_detected['cb_list'] and self._current_ss and not self.chatter_chan_detected(self._current_ss.subscribers, '/listener'):
                        lost_subscriber_detected['cb_list'] = True
                    if not lost_subscriber_detected['cb_diff'] and self._lost_ss and self.chatter_chan_detected(self._lost_ss.subscribers, '/listener'):
                        lost_subscriber_detected['cb_diff'] = True

                if lost_subscriber_detected['list'] and lost_subscriber_detected['diff'] and lost_subscriber_detected['cb_list'] and (lost_subscriber_detected['cb_diff'] or not self.proxy.diff_opt):
                    break
                time.sleep(0.2)

        assert lost_subscriber_detected['list']
        assert lost_subscriber_detected['diff']
        assert lost_subscriber_detected['cb_list']
        assert (self.proxy.diff_opt and lost_subscriber_detected['cb_diff']) or (not self.proxy.diff_opt and not lost_subscriber_detected['cb_diff'])
        time.sleep(0.2)
        # asserting in proxy as well
        assert self.equalMasterSystemState(self.proxy.getSystemState())
        assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())

    def test_detect_subscriber_added_lost_unregister(self):
        def dummy_cb(data):
            pass

        # Start a dummy subscriber
        string_sub = rospy.Subscriber('/test/string', std_msgs.String, dummy_cb)
        try:
            added_subscriber_detected = {'list': False, 'diff': False, 'cb_list': False, 'cb_diff': False}

            # Loop a bit so we can detect the topic
            with timeout(5/self.node_default_spin_freq) as t:
                while not t.timed_out:
                    # Here we only check the last message received
                    if not added_subscriber_detected['list'] and self.conn_list_msgq and self.string_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/test_connection_cache_node'):  # if we find it
                        added_subscriber_detected['list'] = True

                    if not added_subscriber_detected['diff'] and self.conn_diff_msgq and self.string_detected(self.conn_diff_msgq[-1].added, rocon_python_comms.SUBSCRIBER, '/test_connection_cache_node'):  # if we find it
                        added_subscriber_detected['diff'] = True

                    # asserting in proxy callback as well
                    with self._ss_lock:
                        if not added_subscriber_detected['cb_list'] and self._current_ss and self.string_chan_detected(self._current_ss.subscribers, '/test_connection_cache_node'):
                            added_subscriber_detected['cb_list'] = True
                        if not added_subscriber_detected['cb_diff'] and self._added_ss and self.string_chan_detected(self._added_ss.subscribers, '/test_connection_cache_node'):
                            added_subscriber_detected['cb_diff'] = True

                    if added_subscriber_detected['list'] and added_subscriber_detected['diff'] and added_subscriber_detected['cb_list'] and (added_subscriber_detected['cb_diff'] or not self.proxy.diff_opt):
                        break
                    time.sleep(0.2)

            assert added_subscriber_detected['list']
            assert added_subscriber_detected['diff']
            assert added_subscriber_detected['cb_list']
            assert (self.proxy.diff_opt and added_subscriber_detected['cb_diff']) or (not self.proxy.diff_opt and not added_subscriber_detected['cb_diff'])

            # asserting in proxy as well
            time.sleep(0.2)
            assert self.equalMasterSystemState(self.proxy.getSystemState())
            assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

            # Start a dummy sub to trigger a change
            empty_sub = rospy.Subscriber('/test/empty', std_msgs.Empty, dummy_cb)
            try:
                still_subscriber_detected = {'list': False, 'cb_list': False}

                # Loop a bit so we can detect the topic
                with timeout(5/self.node_default_spin_freq) as t:
                    while not t.timed_out:
                        # Here we only check the last message received
                        if not still_subscriber_detected['list'] and self.conn_list_msgq and self.string_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/test_connection_cache_node'):  # if we find it
                            still_subscriber_detected['list'] = True

                        # asserting in proxy callback as well
                        with self._ss_lock:
                            if not still_subscriber_detected['cb_list'] and self._current_ss and self.string_chan_detected(self._current_ss.subscribers, '/test_connection_cache_node'):
                                still_subscriber_detected['cb_list'] = True

                        if still_subscriber_detected['list'] and still_subscriber_detected['cb_list']:
                            break
                        time.sleep(0.2)

                assert still_subscriber_detected['list']
                assert still_subscriber_detected['cb_list']
                time.sleep(0.2)
                # asserting in proxy as well
                assert self.equalMasterSystemState(self.proxy.getSystemState())
                assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())
            finally:
                empty_sub.unregister()

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

        finally:
            string_sub.unregister()

        lost_subscriber_detected = {'list': False, 'diff': False, 'cb_list': False, 'cb_diff': False}

        # Loop a bit so we can detect the topic is gone
        with timeout(5/self.node_default_spin_freq) as t:
            while not t.timed_out:
                # Here we only check the last message received
                if not lost_subscriber_detected['list'] and self.conn_list_msgq and not self.string_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/test_connection_cache_node'):  # if we DONT find it
                    lost_subscriber_detected['list'] = True

                if not lost_subscriber_detected['diff'] and self.conn_diff_msgq and self.string_detected(self.conn_diff_msgq[-1].lost, rocon_python_comms.SUBSCRIBER, '/test_connection_cache_node'):  # if we find it
                    lost_subscriber_detected['diff'] = True

                # asserting in proxy callback as well
                with self._ss_lock:
                    if not lost_subscriber_detected['cb_list'] and self._current_ss and not self.string_chan_detected(self._current_ss.subscribers, '/test_connection_cache_node'):
                        lost_subscriber_detected['cb_list'] = True
                    if not lost_subscriber_detected['cb_diff'] and self._lost_ss and self.string_chan_detected(self._lost_ss.subscribers, '/test_connection_cache_node'):
                        lost_subscriber_detected['cb_diff'] = True

                if lost_subscriber_detected['list'] and lost_subscriber_detected['diff'] and lost_subscriber_detected['cb_list'] and (lost_subscriber_detected['cb_diff'] or not self.proxy.diff_opt):
                    break
                time.sleep(0.2)

        assert lost_subscriber_detected['list']
        assert lost_subscriber_detected['diff']
        assert lost_subscriber_detected['cb_list']
        assert (self.proxy.diff_opt and lost_subscriber_detected['cb_diff']) or (not self.proxy.diff_opt and not lost_subscriber_detected['cb_diff'])
        time.sleep(0.2)
        # asserting in proxy as well
        assert self.equalMasterSystemState(self.proxy.getSystemState())
        assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())

    def test_detect_service_added_lost(self):
        # Start a dummy node
        server_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
        process = self.launch.launch(server_node)
        try:
            added_service_detected = {'list': False, 'diff': False, 'cb_list': False, 'cb_diff': False}

            # Loop a bit so we can detect the service
            with timeout(5/self.node_default_spin_freq) as t:
                while not t.timed_out:
                    # Here we only check the last message received
                    if not added_service_detected['list'] and self.conn_list_msgq and self.add_two_ints_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SERVICE, '/add_two_ints_server'):  # if we find it
                        added_service_detected['list'] = True

                    if not added_service_detected['diff'] and self.conn_diff_msgq and self.add_two_ints_detected(self.conn_diff_msgq[-1].added, rocon_python_comms.SERVICE, '/add_two_ints_server'):  # if we find it
                        added_service_detected['diff'] = True

                    # asserting in proxy callback as well
                    with self._ss_lock:
                        if not added_service_detected['cb_list'] and self._current_ss and self.add_two_ints_chan_detected(self._current_ss.services, '/add_two_ints_server'):
                            added_service_detected['cb_list'] = True
                        if not added_service_detected['cb_diff'] and self._added_ss and self.add_two_ints_chan_detected(self._added_ss.services, '/add_two_ints_server'):
                            added_service_detected['cb_diff'] = True

                    if added_service_detected['list'] and added_service_detected['diff'] and added_service_detected['cb_list'] and (added_service_detected['cb_diff'] or not self.proxy.diff_opt):
                        break
                    time.sleep(0.2)

            assert added_service_detected['list']
            assert added_service_detected['diff']
            assert added_service_detected['cb_list']
            assert (self.proxy.diff_opt and added_service_detected['cb_diff']) or (not self.proxy.diff_opt and not added_service_detected['cb_diff'])
            time.sleep(0.2)
            # asserting in proxy as well
            assert self.equalMasterSystemState(self.proxy.getSystemState())
            assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())

            still_service_detected = {'list': False, 'cb_list': False}

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

            # Start a dummy node
            talker_node = roslaunch.core.Node('roscpp_tutorials', 'talker')
            distraction_process = self.launch.launch(talker_node)
            try:
                # Loop a bit so we can detect the service
                with timeout(5/self.node_default_spin_freq) as t:
                    while not t.timed_out:
                        # Here we only check the last message received
                        if not still_service_detected['list'] and self.conn_list_msgq and self.add_two_ints_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SERVICE, '/add_two_ints_server'):  # if we find it
                            still_service_detected['list'] = True

                        # asserting in proxy callback as well
                        with self._ss_lock:
                            if not still_service_detected['cb_list'] and self._current_ss and self.add_two_ints_chan_detected(self._current_ss.services, '/add_two_ints_server'):
                                still_service_detected['cb_list'] = True

                        if still_service_detected['list'] and still_service_detected['cb_list']:
                            break
                        time.sleep(0.2)

                assert still_service_detected['list']
                assert still_service_detected['cb_list']
                time.sleep(0.2)
                # asserting in proxy as well
                assert self.equalMasterSystemState(self.proxy.getSystemState())
            finally:
                distraction_process.stop()

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()

        finally:
            process.stop()

        lost_service_detected = {'list': False, 'diff': False, 'cb_list': False, 'cb_diff': False}

        # Loop a bit so we can detect the service is gone
        with timeout(5/self.node_default_spin_freq) as t:
            while not t.timed_out:
                # Here we only check the last message received
                if not lost_service_detected['list'] and self.conn_list_msgq and not self.add_two_ints_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SERVICE, '/add_two_ints_server'):  # if we DONT find it
                    lost_service_detected['list'] = True

                if not lost_service_detected['diff'] and self.conn_diff_msgq and self.add_two_ints_detected(self.conn_diff_msgq[-1].lost, rocon_python_comms.SERVICE, '/add_two_ints_server'):  # if we find it
                    lost_service_detected['diff'] = True

                # asserting in proxy callback as well
                with self._ss_lock:
                    if not lost_service_detected['cb_list'] and self._current_ss and not self.add_two_ints_chan_detected(self._current_ss.services, '/add_two_ints_server'):
                        lost_service_detected['cb_list'] = True
                    if not lost_service_detected['cb_diff'] and self._lost_ss and self.add_two_ints_chan_detected(self._lost_ss.services, '/add_two_ints_server'):
                        lost_service_detected['cb_diff'] = True

                if lost_service_detected['list'] and lost_service_detected['diff'] and lost_service_detected['cb_list'] and (lost_service_detected['cb_diff'] or not self.proxy.diff_opt):
                    break
                time.sleep(0.2)

        assert lost_service_detected['list']
        assert lost_service_detected['diff']
        assert lost_service_detected['cb_list']
        assert (self.proxy.diff_opt and lost_service_detected['cb_diff']) or (not self.proxy.diff_opt and not lost_service_detected['cb_diff'])

        time.sleep(0.2)
        # asserting in proxy as well
        assert self.equalMasterSystemState(self.proxy.getSystemState())
        assert self.equalMasterTopicTypes(self.proxy.getTopicTypes())

    def test_change_spin_rate_detect_sub(self):
        # constant use just to prevent spinning too fast
        overspin_sleep_val = 0.02
        def prevent_overspin_sleep():
            time.sleep(overspin_sleep_val)

        # Prepare launcher
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # wait until we get current connectioncache spin
        with timeout(5) as t:
            while not t.timed_out and self.spin_freq == 0.0:
                prevent_overspin_sleep()

        assert not self.spin_freq == 0.0

        # Make rate 3 times slower ( enough to have time to create the node )
        mem_spin_freq = self.spin_freq
        rate_msg = rocon_std_msgs.ConnectionCacheSpin()
        rate_msg.spin_freq = self.spin_freq/3
        rate_msg.spin_timer = 0.0
        self.set_spin.publish(rate_msg)

        # Start a dummy node
        listener_node = roslaunch.core.Node('roscpp_tutorials', 'listener')
        process = self.launch.launch(listener_node)
        try:
            # check that we dont get any update
            added_subscriber_diff_detected = False

            # wait - only as long as a tick - until rate has been published as changed.
            # during this time we shouldnt detect the subscriber
            counter = 0
            while (counter == 0 or 1/(counter * overspin_sleep_val) < mem_spin_freq) and self.spin_freq == mem_spin_freq:
                counter += 1
                # Here we only check the last message received
                if self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                    added_subscriber_diff_detected = True

                assert not added_subscriber_diff_detected
                prevent_overspin_sleep()

            # we should have waited less than one tick
            assert counter == 0 or 1/(counter * overspin_sleep_val) < mem_spin_freq

            # Start counting
            start_wait = rospy.get_time()
            while not added_subscriber_diff_detected and rospy.get_time() - start_wait < 1/self.spin_freq:
                # check that we get an update
                # Here we only check the last message received
                if self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                    added_subscriber_diff_detected = True
                prevent_overspin_sleep()

            assert added_subscriber_diff_detected

            # Make rate fast again
            last_freq = self.spin_freq
            rate_msg = rocon_std_msgs.ConnectionCacheSpin()
            rate_msg.spin_freq = mem_spin_freq
            rate_msg.spin_timer = 0.0
            self.set_spin.publish(rate_msg)

        finally:
            process.stop()

        # restart the dummy node
        process = self.launch.launch(listener_node)
        try:
            added_subscriber_diff_detected = False

            # wait - only for a tick - until rate has changed
            counter = 0
            while (counter == 0 or 1/(counter * overspin_sleep_val) < last_freq) and not self.spin_freq == mem_spin_freq:
                counter += 1
                prevent_overspin_sleep()

            # we should have waited less than one tick
            assert counter == 0 or 1/(counter * overspin_sleep_val) < last_freq

            # Start counting
            start_wait = rospy.get_time()
            while not added_subscriber_diff_detected and rospy.get_time() - start_wait < 1/mem_spin_freq:
                # check that we get an update
                # Here we only check the last message received
                if self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                    added_subscriber_diff_detected = True

            assert added_subscriber_diff_detected
        finally:
            process.stop()

    def test_change_spin_rate_timer_detect_sub(self):
        # constant use just to prevent spinning too fast
        overspin_sleep_val= 0.02
        def prevent_overspin_sleep():
            time.sleep(overspin_sleep_val)

        # Prepare launcher
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # wait until we get current connectioncache spin
        with timeout(5) as t:
            while not t.timed_out and self.spin_freq == 0.0:
                prevent_overspin_sleep()

        assert not self.spin_freq == 0.0

        # Make rate 3 times slower ( enough to have time to create the node )
        # temporarily
        mem_spin_freq = self.spin_freq
        rate_msg = rocon_std_msgs.ConnectionCacheSpin()
        rate_msg.spin_freq = self.spin_freq/3
        # Waiting just a bit (1 secs) more than the test time before we get back to speed
        rate_msg.spin_timer = 1/mem_spin_freq + 1/self.spin_freq + 1.0
        self.set_spin.publish(rate_msg)

        # Start a dummy node
        listener_node = roslaunch.core.Node('roscpp_tutorials', 'listener')
        process = self.launch.launch(listener_node)
        try:
            # check that we dont get any update
            added_subscriber_diff_detected = False

            # wait - only as long as a tick - until rate has been published as changed.
            # during this time we shouldnt detect the subscriber
            counter = 0
            while (counter == 0 or 1/(counter * overspin_sleep_val) < mem_spin_freq) and self.spin_freq == mem_spin_freq:
                counter += 1
                # Here we only check the last message received
                if self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                    added_subscriber_diff_detected = True

                assert not added_subscriber_diff_detected
                prevent_overspin_sleep()

            # we should have waited less than one tick
            assert counter == 0 or 1/(counter * overspin_sleep_val) < mem_spin_freq

            # Start counting
            start_wait = rospy.get_time()
            while not added_subscriber_diff_detected and rospy.get_time() - start_wait < 1/self.spin_freq:
                # check that we get an update
                # Here we only check the last message received
                if self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                    added_subscriber_diff_detected = True
                prevent_overspin_sleep()

            assert added_subscriber_diff_detected

            # the spin rate should still be set here
            assert self.spin_timer > 0.0
            assert self.spin_freq < mem_spin_freq

            # Wait for timer to finish and rate to get fast again
            with timeout(rate_msg.spin_timer) as t:
                while not t.timed_out and (self.spin_timer > 0.0 or self.spin_freq < mem_spin_freq):
                    prevent_overspin_sleep()

            assert self.spin_timer == 0.0
            assert self.spin_freq == mem_spin_freq
        finally:
            process.stop()

        # restart the dummy node
        process = self.launch.launch(listener_node)
        try:
            added_subscriber_diff_detected = False

            # Start counting
            start_wait = rospy.get_time()
            while not added_subscriber_diff_detected and rospy.get_time() - start_wait < 1/mem_spin_freq:
                # check that we get an update
                # Here we only check the last message received
                if self.conn_list_msgq and self.chatter_detected(self.conn_list_msgq[-1].connections, rocon_python_comms.SUBSCRIBER, '/listener'):  # if we find it
                    added_subscriber_diff_detected = True

            assert added_subscriber_diff_detected
        finally:
            process.stop()


class TestConnectionCacheNodeDiff(TestConnectionCacheNode):

    def setUp(self, cacheproxy=rocon_python_comms.ConnectionCacheProxy):
        super(TestConnectionCacheNodeDiff, self).setUp(partial(rocon_python_comms.ConnectionCacheProxy, user_callback=self.user_cb, diff_opt=True))

    def tearDown(self):
        super(TestConnectionCacheNodeDiff, self).tearDown()


if __name__ == '__main__':

    # setup_module()
    rostest.rosrun('rocon_python_comms',
                   'test_connection_cache_node',
                   TestConnectionCacheNode)
    # teardown_module()
