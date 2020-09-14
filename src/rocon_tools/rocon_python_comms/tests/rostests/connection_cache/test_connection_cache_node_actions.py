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


class TestConnectionCacheNodeActions(unittest.TestCase):
    launch = None

    @classmethod
    def setUpClass(cls):

        # initialize the launch API first
        cls.launch = roslaunch.scriptapi.ROSLaunch()
        cls.launch.start()

        # Note this should be called only once by process.
        # We only run one process here, for all tests
        rospy.init_node('test_connection_cache_node_actions')

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
        cacheproxy = cacheproxy or partial(rocon_python_comms.ConnectionCacheProxy, user_callback=self.user_cb, diff_opt=False, handle_actions=True)
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

    def fibonacci_action_chan_detected(self, actionq_cdict, node_name):
        """
        detect the fibonacci action server in a chan list
        """
        test = '/fibonacci' in actionq_cdict.keys() and (
            actionq_cdict['/fibonacci'].name == '/fibonacci'
            and actionq_cdict['/fibonacci'].type == 'actionlib_tutorials/FibonacciActionGoal'
            and len([n for n in actionq_cdict['/fibonacci'].nodes if n[0].startswith(node_name)]) > 0  # sometime the node gets suffixes with uuid
        )
        if not test:
            print "Expected : name:{name} type:{type} node:{node}".format(name='/fibonacci', node=node_name, type='actionlib_tutorials/FibonacciActionGoal')
            print "NOT FOUND IN DICT : {0}".format(actionq_cdict)
        return test

    def test_detect_action_server_added_lost(self,):
        # Start a dummy node
        action_server_node = roslaunch.core.Node('actionlib_tutorials', 'fibonacci_server')
        process = self.launch.launch(action_server_node)
        try:
            added_action_server_detected = {'cb_list': False, 'cb_diff': False}

            # Loop a bit so we can detect the action
            with timeout(5/self.node_default_spin_freq) as t:
                while not t.timed_out:
                    # asserting in proxy callback only ( action filtering is happening in proxy )
                    with self._ss_lock:
                        if not added_action_server_detected['cb_list'] and self._current_ss and self.fibonacci_action_chan_detected(self._current_ss.action_servers, '/fibonacci_server'):
                            added_action_server_detected['cb_list'] = True
                        if self.proxy.diff_opt and self._added_ss and self.fibonacci_action_chan_detected(self._added_ss.action_servers, '/fibonacci_server'):
                            added_action_server_detected['cb_diff'] = True

                    if added_action_server_detected['cb_list'] and (added_action_server_detected['cb_diff'] or not self.proxy.diff_opt):
                        break
                    time.sleep(0.2)

            assert added_action_server_detected['cb_list']
            assert (self.proxy.diff_opt and added_action_server_detected['cb_diff']) or (not self.proxy.diff_opt and not added_action_server_detected['cb_diff'])

            time.sleep(0.2)

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

            # Start a dummy node to trigger a change
            server_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
            distraction_process = self.launch.launch(server_node)
            try:
                still_action_server_detected = {'cb_list': False}

                # Loop a bit so we can detect the topic
                with timeout(5/self.node_default_spin_freq) as t:
                    while not t.timed_out:
                        # asserting in proxy callback only
                        with self._ss_lock:
                            if not still_action_server_detected['cb_list'] and self._current_ss and self.fibonacci_action_chan_detected(self._current_ss.action_servers, '/fibonacci_server'):
                                still_action_server_detected['cb_list'] = True

                        if still_action_server_detected['cb_list']:
                            break
                        time.sleep(0.2)

                assert still_action_server_detected['cb_list']
                time.sleep(0.2)
            finally:
                distraction_process.stop()

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

        finally:
            process.stop()

        lost_action_server_detected = {'cb_list': False, 'cb_diff': False}

        # Loop a bit so we can detect the topic is gone
        with timeout(5/self.node_default_spin_freq) as t:
            while not t.timed_out:
                # asserting in proxy callback only
                with self._ss_lock:
                    if not lost_action_server_detected['cb_list'] and self._current_ss and not self.fibonacci_action_chan_detected(self._current_ss.action_servers, '/fibonacci_server'):
                        lost_action_server_detected['cb_list'] = True
                    if not lost_action_server_detected['cb_diff'] and self._lost_ss and self.fibonacci_action_chan_detected(self._lost_ss.action_servers, '/fibonacci_server'):
                        lost_action_server_detected['cb_diff'] = True

                if lost_action_server_detected['cb_list'] and (lost_action_server_detected['cb_diff'] or not self.proxy.diff_opt):
                    break
                time.sleep(0.2)

        assert lost_action_server_detected['cb_list']
        assert (self.proxy.diff_opt and lost_action_server_detected['cb_diff']) or (not self.proxy.diff_opt and not lost_action_server_detected['cb_diff'])
        time.sleep(0.2)

    def test_detect_action_client_added_lost(self):
        # Start a dummy node
        action_client_node = roslaunch.core.Node('actionlib_tutorials', 'fibonacci_client')
        process = self.launch.launch(action_client_node)
        try:
            added_action_client_detected = {'cb_list': False, 'cb_diff': False}

            # Loop a bit so we can detect the topic
            with timeout(100/self.node_default_spin_freq) as t:
                while not t.timed_out:
                    # asserting in proxy callback only
                    with self._ss_lock:
                        if not added_action_client_detected['cb_list'] and self._current_ss and self.fibonacci_action_chan_detected(self._current_ss.action_clients, '/fibonacci_client'):
                            added_action_client_detected['cb_list'] = True
                        if not added_action_client_detected['cb_diff'] and self._added_ss and self.fibonacci_action_chan_detected(self._added_ss.action_clients, '/fibonacci_client'):
                            added_action_client_detected['cb_diff'] = True

                    if added_action_client_detected['cb_list'] and (added_action_client_detected['cb_diff'] or not self.proxy.diff_opt):
                        break
                    time.sleep(0.2)

            assert added_action_client_detected['cb_list']
            assert (self.proxy.diff_opt and added_action_client_detected['cb_diff']) or (not self.proxy.diff_opt and not added_action_client_detected['cb_diff'])

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

            # Start a dummy node to trigger a change
            server_node = roslaunch.core.Node('roscpp_tutorials', 'add_two_ints_server')
            distraction_process = self.launch.launch(server_node)
            try:
                still_subscriber_detected = {'cb_list': False}

                # Loop a bit so we can detect the topic
                with timeout(5/self.node_default_spin_freq) as t:
                    while not t.timed_out:
                        # asserting in proxy callback only
                        with self._ss_lock:
                            if not still_subscriber_detected['cb_list'] and self._current_ss and self.fibonacci_action_chan_detected(self._current_ss.action_clients, '/fibonacci_client'):
                                still_subscriber_detected['cb_list'] = True

                        if still_subscriber_detected['cb_list']:
                            break
                        time.sleep(0.2)

                assert still_subscriber_detected['cb_list']
                time.sleep(0.2)
            finally:
                distraction_process.stop()

            # clean list messages to make sure we get new ones
            self.conn_list_msgq = deque()
            self.conn_diff_msgq = deque()
            self.cleanup_user_cb_ss()

        finally:
            process.stop()

        lost_action_client_detected = { 'cb_list': False, 'cb_diff': False}

        # Loop a bit so we can detect the topic is gone
        with timeout(5/self.node_default_spin_freq) as t:
            while not t.timed_out:
                # asserting in proxy callback only
                with self._ss_lock:
                    if not lost_action_client_detected['cb_list'] and self._current_ss and not self.fibonacci_action_chan_detected(self._current_ss.action_clients, '/fibonacci_client'):
                        lost_action_client_detected['cb_list'] = True
                    if not lost_action_client_detected['cb_diff'] and self._lost_ss and self.fibonacci_action_chan_detected(self._lost_ss.action_clients, '/fibonacci_client'):
                        lost_action_client_detected['cb_diff'] = True

                if  lost_action_client_detected['cb_list'] and (lost_action_client_detected['cb_diff'] or not self.proxy.diff_opt):
                    break
                time.sleep(0.2)

        assert lost_action_client_detected['cb_list']
        assert (self.proxy.diff_opt and lost_action_client_detected['cb_diff']) or (not self.proxy.diff_opt and not lost_action_client_detected['cb_diff'])
        time.sleep(0.2)


class TestConnectionCacheNodeDiffActions(TestConnectionCacheNodeActions):

    def setUp(self, cacheproxy=rocon_python_comms.ConnectionCacheProxy):
        super(TestConnectionCacheNodeDiffActions, self).setUp(partial(rocon_python_comms.ConnectionCacheProxy, user_callback=self.user_cb, diff_opt=True, handle_actions=True))

    def tearDown(self):
        super(TestConnectionCacheNodeDiffActions, self).tearDown()


if __name__ == '__main__':

    # setup_module()
    rostest.rosrun('rocon_python_comms',
                   'test_connection_cache_node_actions',
                   TestConnectionCacheNodeActions)
    # teardown_module()
