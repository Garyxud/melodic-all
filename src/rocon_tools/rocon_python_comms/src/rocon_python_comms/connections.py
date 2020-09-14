#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
##############################################################################
# Description
##############################################################################

"""
.. module:: connections
   :platform: Unix
   :synopsis: A comprehensive api for listing/handling ros connections.


This is a wrapper around the many ad-hoc modules that work with the ros master
system state list of pubs, subs, services and actions. In some cases it
just extends the functionality (severely lacking in some cases) and in others
it provides new, higher level methods (e.g. for actions).
----

"""

##############################################################################
# Imports
##############################################################################
import copy
import os
import re
import socket
import threading

import collections
import time

import rocon_std_msgs.msg as rocon_std_msgs
import rosgraph
import rospy
import rostopic
import rosservice

##############################################################################
# Aliases
##############################################################################

# Can't see an easier way to alias or import these
PUBLISHER = rocon_std_msgs.Connection.PUBLISHER
SUBSCRIBER = rocon_std_msgs.Connection.SUBSCRIBER
SERVICE = rocon_std_msgs.Connection.SERVICE
ACTION_SERVER = rocon_std_msgs.Connection.ACTION_SERVER
ACTION_CLIENT = rocon_std_msgs.Connection.ACTION_CLIENT

##############################################################################
# Constants
##############################################################################

# for help in iterating over the set of connection constants
connection_types = frozenset([PUBLISHER,
                              SUBSCRIBER,
                              SERVICE,
                              ])
connection_types_actions = frozenset([
                              ACTION_CLIENT,
                              ACTION_SERVER
                              ])
connection_types_list = [PUBLISHER,
                         SUBSCRIBER,
                         SERVICE,
                         ]
connection_types_actions_list = [
                         ACTION_CLIENT,
                         ACTION_SERVER
                         ]

action_types = ['/goal', '/cancel', '/status', '/feedback', '/result']

##############################################################################
# Classes
##############################################################################


class Connection(object):
    """
      An object that represents a connection containing all the gory details
      about a connection, allowing a connection to be passed along to other nodes.

      Note, we use a ros msg type as a data structure for the variable storage.
      This lets users spin it off in the ros world as needed as well as
      providing extra operators for manipulation and handling of connection
      types on top.
    """

    def __init__(self, connection_type, name, node, type_msg=None, type_info=None, xmlrpc_uri=None):
        """
        :param str type: type of connection from string constants in rocon_std_msgs.Connection (e.g. PUBLISHER)
        :param str name: the topic/service name or the action base name
        :param str node: the name of the node establishing this connection
        :param str type_msg: topic or service type, e.g. std_msgs/String
        :param str type_info: extra type information ( following rospy implementation ) : service uri or topic type
        :param str xmlrpc_uri: xmlrpc node uri for managing the connection
        """
        self._connection = rocon_std_msgs.Connection(connection_type, name, node, type_msg, type_info, xmlrpc_uri)

    @property
    def type(self):
        return self._connection.type if self._connection.type else None

    @type.setter
    def type(self, connection_type):
        self._connection.type = connection_type if connection_type else ""

    @property
    def name(self):
        return self._connection.name if self._connection.name else None

    @name.setter
    def name(self, connection_name):
        self._connection.name = connection_name if connection_name else ""

    @property
    def node(self):
        return self._connection.node if self._connection.node else None

    @node.setter
    def node(self, connection_node):
        self._connection.node = connection_node if connection_node else ""

    @property
    def type_msg(self):
        return self._connection.type_msg if self._connection.type_msg else None

    @type_msg.setter
    def type_msg(self, connection_type_msg):
        self._connection.type_msg = connection_type_msg if connection_type_msg else ""

    @property
    def type_info(self):
        return self._connection.type_info if self._connection.type_info else None

    @type_info.setter
    def type_info(self, connection_type_info):
        self._connection.type_info = connection_type_info if connection_type_info else ""

    @property
    def xmlrpc_uri(self):
        return self._connection.xmlrpc_uri if self._connection.xmlrpc_uri else None

    @xmlrpc_uri.setter
    def xmlrpc_uri(self, connection_xmlrpc_uri):
        self._connection.xmlrpc_uri = connection_xmlrpc_uri if connection_xmlrpc_uri else ""

    @property
    def msg(self):
        return self._connection

    @msg.setter
    def msg(self, msg):
        self._connection = msg

    def generate_type_info_msg(self):
        """
        Basic connection details are provided by get system state from the master, which is
        a one shot call to give you information about every connection possible. it does
        not however provide type info information and the only way of retrieving that from
        the master is making one xmlrpc call to the master for every single connection.
        This gets expensive, so generating this information is usually delayed until we
        need it and done via this method.
        """
        if self.type_info is None:
            if self.type == PUBLISHER or self.type == SUBSCRIBER:
                try:
                    self.type_info = rostopic.get_topic_type(self.name)[0]  # message type
                    self.type_msg = self.type_info
                except rostopic.ROSTopicIOException as topic_exc:
                    rospy.logwarn(topic_exc)
            elif self.type == SERVICE:
                try:
                    self.type_info = rosservice.get_service_uri(self.name)
                    self.type_msg = rosservice.get_service_type(self.name)
                except rosservice.ROSServiceIOException as service_exc:
                    rospy.logwarn(service_exc)
            elif self.type == ACTION_SERVER or self.type == ACTION_CLIENT:
                try:
                    goal_topic = self.name + '/goal'
                    goal_topic_type = rostopic.get_topic_type(goal_topic)
                    self.type_info = re.sub('ActionGoal$', '', goal_topic_type[0])  # Base type for action
                    self.type_msg = self.type_info
                except rostopic.ROSTopicIOException as topic_exc:
                    rospy.logwarn(topic_exc.msg)
        return self  # chaining

    def generate_xmlrpc_info(self, master=None):
        """
        As with type info, detailed xmlrpc info has to be generated on a per connection
        basis which is expensive, so it's best to delay its generation until needed.

        :param rosgraph.Master master : if you've already got a master xmlrpc client initialised, use that.
        """
        if self.xmlrpc_uri is None:
            if master is None:
                master = rosgraph.Master(self.node)
            try:
                self.xmlrpc_uri = master.lookupNode(self.node)
            except rosgraph.MasterError as exc:
                rospy.logwarn(str(exc))  # keep going even if the node is not found.
        return self  # chaining

    def __eq__(self, other):
        """
          Don't need to check every characteristic of the connection to
          uniquely identify it, just the name, node and type.
        """
        if isinstance(other, self.__class__):
            # return self.__dict__ == other.__dict__
            return (self.name == other.name and
                    self.type == other.type and
                    self.node == other.node)
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        """
        String representation of the connection, it differs a little by connection type.
        """
        if self.type == SERVICE:
            return '{type: %s, name: %s, node: %s, uri: %s, service_api: %s}' % (self.type,
                                                                                 self.name,
                                                                                 self.node,
                                                                                 self.xmlrpc_uri,
                                                                                 self.type_info
                                                                                 )
        else:
            return '{type: %s, name: %s, node: %s, uri: %s, topic_type: %s}' % (self.type,
                                                                                self.name,
                                                                                self.node,
                                                                                self.xmlrpc_uri,
                                                                                self.type_info
                                                                                )

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return hash((self.name, self.type, self.node))


##############################################################################
# Utility Methods
##############################################################################


def create_connection(ConnectionMsg):
    """
    Creates a Connection instance from a Connection message
    """
    return Connection(ConnectionMsg.type, ConnectionMsg.name, ConnectionMsg.node, ConnectionMsg.type_msg, ConnectionMsg.type_info, ConnectionMsg.xmlrpc_uri)


def create_empty_connection_type_dictionary(types=None):
    '''
      Used to initialise a dictionary with rule type keys
      and empty lists.
    '''
    types = types or connection_types
    dic = {}
    for connection_type in types:
        dic[connection_type] = set()
    return dic

##############################################################################
# Connection Cache
##############################################################################


class ConnectionCache(object):
    """
    Caches all of the connections living in a ros master. Use the 'update'
    method to force a refresh of the basic information for every connection.
    """

    def __init__(self):
        #: rosgraph.Master: master API instance
        self.master = rosgraph.Master(rospy.get_name())

        #: dict: dict structure of connections, by type.
        self.connections = create_empty_connection_type_dictionary(connection_types)

    def find(self, name):
        """
        Convenience function for finding all connections with the
        specified name.

        @TODO other find methods using a mix of name, node, type.
        """
        types = connection_types
        found_connections = []
        for connection_type in types:
            for connection in self.connections[connection_type]:
                if name == connection.name:
                    found_connections.append(connection)
        return found_connections

    def __str__(self):
        """
        String representation of the connection cache.
        """
        s = ""
        types = connection_types
        for connection_type in types:
            s += ("%s:\n" % connection_type)
            for connection in self.connections[connection_type]:
                s += "  {name: %s, node: %s, type_info: %s, xmlrpc_uri: %s}\n" % (connection.name,
                                                                                  connection.node,
                                                                                  connection.type_info,
                                                                                  connection.xmlrpc_uri
                                                                                  )
        return s

    def update(self, new_system_state=None, new_topic_types=None):
        """
          Currently completely regenerating the connections dictionary and then taking
          diffs. Could be faster if we took diffs on the system state instead, but that's
          a bit more awkward since each element has a variable list of nodes that we'd have
          to check against to get good diffs. e.g.

            old_publishers = ['/chatter', ['/talker']]
            new_publishers = ['/chatter', ['/talker', '/babbler']]
        """
        # init the variables we will return
        new_connections = create_empty_connection_type_dictionary(connection_types)
        lost_connections = create_empty_connection_type_dictionary(connection_types)

        if new_system_state is None:
            try:
                publishers, subscribers, services = self.master.getSystemState()
                topic_types = self.master.getTopicTypes()
            except socket.error:
                rospy.logerr("ConnectionCache : couldn't get system state from the master "
                             "[did you set your master uri to a wireless IP that just went down?]")
                return new_connections, lost_connections
        else:
            publishers = new_system_state[PUBLISHER]
            subscribers = new_system_state[SUBSCRIBER]
            services = new_system_state[SERVICE]
            topic_types = new_topic_types

        pubs = self._get_connections_from_pub_sub_list(publishers, PUBLISHER, topic_types)
        new_connections[PUBLISHER] = pubs - self.connections[PUBLISHER]
        for c in new_connections[PUBLISHER]:
            c.generate_xmlrpc_info()
        # lost connections already have xmlrpc_uri and it s not checked by set for unicity (__hash__)
        lost_connections[PUBLISHER] = self.connections[PUBLISHER] - pubs

        subs = self._get_connections_from_pub_sub_list(subscribers, SUBSCRIBER, topic_types)
        new_connections[SUBSCRIBER] = subs - self.connections[SUBSCRIBER]
        for c in new_connections[SUBSCRIBER]:
            c.generate_xmlrpc_info()
        # lost connections already have xmlrpc_uri and it s not checked by set for unicity (__hash__)
        lost_connections[SUBSCRIBER] = self.connections[SUBSCRIBER] - subs

        svcs = self._get_connections_from_service_list(services, SERVICE)
        new_connections[SERVICE] = svcs - self.connections[SERVICE]
        for c in new_connections[SERVICE]:
            c.generate_type_info_msg()
            c.generate_xmlrpc_info()
        # lost connections already have xmlrpc_uri and it s not checked by set for unicity (__hash__)
        # type_info is different but it is also not checked by set for unicity (__hash__)
        lost_connections[SERVICE] = self.connections[SERVICE] - svcs

        self.connections[PUBLISHER].update(new_connections[PUBLISHER])
        self.connections[PUBLISHER].difference_update(lost_connections[PUBLISHER])

        self.connections[SUBSCRIBER].update(new_connections[SUBSCRIBER])
        self.connections[SUBSCRIBER].difference_update(lost_connections[SUBSCRIBER])

        self.connections[SERVICE].update(new_connections[SERVICE])
        self.connections[SERVICE].difference_update(lost_connections[SERVICE])

        return new_connections, lost_connections

    @staticmethod
    def _get_connections_from_service_list(connection_list, connection_type):
        connections = set()
        for service in connection_list:
            service_name = service[0]
            # service_uri = rosservice.get_service_uri(service_name)
            nodes = service[1]
            for node in nodes:
                # try:
                #    node_uri = self.lookupNode(node)
                # except:
                #    continue
                connection = Connection(connection_type, service_name, node)  # service_uri, node_uri
                connections.add(connection)
        return connections

    @staticmethod
    def _get_connections_from_pub_sub_list(connection_list, connection_type, msg_type_list):
        connections = set()
        for topic in connection_list:
            topic_name = topic[0]
            topic_type = [t[1] for t in msg_type_list if t[0] == topic_name]
            topic_type = topic_type[0] if topic_type else None
            nodes = topic[1]
            for node in nodes:
                # try:
                    # node_uri = self.lookupNode(node)
                # except:
                #    continue
                connection = Connection(connection_type, topic_name, node, topic_type, topic_type)
                connections.add(connection)
        return connections

    @staticmethod
    def _get_connections_from_action_list(connection_list, connection_type):
        connections = set()
        for action in connection_list:
            action_name = action[0]
            # goal_topic = action_name + '/goal'
            # goal_topic_type = rostopic.get_topic_type(goal_topic)
            # topic_type = re.sub('ActionGoal$', '', goal_topic_type[0])  # Base type for action
            nodes = action[1]
            for node in nodes:
                # try:
                #    node_uri = self.lookupNode(node)
                # except:
                #    continue
                connection = Connection(connection_type, action_name, node)
                connections.add(connection)
        return connections


class ConnectionCacheNode(object):
    def __init__(self):

        self.spin_freq = rospy.get_param("~spin_freq", 0.1)
        self.spin_original_freq = self.spin_freq
        self.spin_timer = 0.0
        self.conn_cache = ConnectionCache()  # we want a drop in replacement for ROSmaster access

        self.conn_cache_spin_pub = rospy.Publisher("~spin", rocon_std_msgs.ConnectionCacheSpin, latch=True, queue_size=1)
        self.conn_cache_spin_sub = rospy.Subscriber("~spin", rocon_std_msgs.ConnectionCacheSpin, self.set_spin_cb)

        self.conn_list = rospy.Publisher("~list", rocon_std_msgs.ConnectionsList, latch=True, queue_size=1)  # uptodate full list
        self.conn_diff = rospy.Publisher("~diff", rocon_std_msgs.ConnectionsDiff, queue_size=1, tcp_nodelay=True)  # differences only for faster parsing.

    def set_spin_cb(self, data):
        if data.spin_freq and not data.spin_freq == self.spin_freq:  # we change the rate if needed
            self.spin_freq = data.spin_freq
            self.spin_timer = data.spin_timer

    def spin(self):
        rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
        try:
            # sensible default values
            self.spin_rate = rospy.Rate(self.spin_freq)
            last_spinmsg = None
            last_update = time.time()

            while not rospy.core.is_shutdown():
                elapsed_time = time.time() - last_update
                self.spin_timer = max(self.spin_timer - elapsed_time, 0.0)
                last_update = time.time()

                # If needed (or first time) we change our spin rate, and publish the new frequency
                if self.spin_timer > 0.0 or last_spinmsg is None or last_spinmsg.spin_timer > 0.0:
                    # if spin_timer just came back to 0.0 we use self.spin_original_freq
                    if self.spin_timer == 0.0:
                        self.spin_freq = self.spin_original_freq
                    # if timer is almost finished we need to increase rate to be back to original speed on time
                    self.spin_rate = rospy.Rate(
                        self.spin_freq if self.spin_timer == 0.0 else max(self.spin_freq, 1 / self.spin_timer)
                    )
                    spinmsg = rocon_std_msgs.ConnectionCacheSpin()
                    spinmsg.spin_freq = self.spin_freq
                    spinmsg.spin_timer = self.spin_timer
                    last_spinmsg = spinmsg
                    self.conn_cache_spin_pub.publish(spinmsg)

                try:
                    new_conns, lost_conns = self.conn_cache.update()
                    changed = False

                    diff_msg = rocon_std_msgs.ConnectionsDiff()
                    list_msg = rocon_std_msgs.ConnectionsList()
                    for ct in connection_types:
                        if new_conns[ct] or lost_conns[ct]:  # something changed
                            changed = True
                            for c in new_conns[ct]:
                                create_connection(c)
                                diff_msg.added.append(c.msg)
                            for c in lost_conns[ct]:
                                create_connection(c)
                                diff_msg.lost.append(c.msg)
                        # we always need all connections types in the full list
                        for c in self.conn_cache.connections[ct]:
                            create_connection(c)
                            list_msg.connections.append(c.msg)

                    if changed:
                        # rospy.loginfo("COMPLETE LIST : {0}".format(self.conn_cache.connections))
                        # rospy.loginfo("NEW : {0}".format(new_conns))
                        # rospy.loginfo("LOST : {0}".format(lost_conns))

                        self.conn_diff.publish(diff_msg)  # new_conns, old_conns
                        self.conn_list.publish(list_msg)  # conn_cache.connections

                except rospy.ROSException:
                    rospy.logerr("ROS Watcher : Connections list unavailable.")
                except rospy.ROSInterruptException:
                    rospy.logerr("ROS Watcher : ros shutdown while looking for Connections .")

                self.spin_rate.sleep()

        except KeyboardInterrupt:
            rospy.logdebug("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')


# TODO : split that in multiple files
class ConnectionCacheProxy(object):
    class InitializationTimeout(Exception):
        pass

    class Channel(object):
        """
        Definition of a channel ( topic/service )
        => a compressed version of a list of connection with same name
        """

        def __init__(self, name, type, xmlrpc_uri, nodes=None):
            """
            Initialize a Channel instance
            :param name
            :param type
            :param nodes a set of tuple (node_name, node_uri)
            """
            self.name = name
            self.type = type
            # ROS master keeps only one URI per service ( only the last node service URI is kept )
            # So we actually have only one xmlrpc_uri per channel
            self.xmlrpc_uri = xmlrpc_uri
            self.nodes = nodes or set()

        def __eq__(self, other):  # used for manual == operator
            if not isinstance(other, ConnectionCacheProxy.Channel):
                return NotImplemented
            elif self is other:
                return True
            else:
                return (self.name == other.name and
                        self.type == other.type and
                        self.xmlrpc_uri == other.xmlrpc_uri and
                        self.nodes == other.nodes)

        def __hash__(self):  # used for comparison in sets
            return hash((self.name, self.type))

        @staticmethod
        def dict_factory(conn_list, chan_dict=None):
            """
            Merge a list of Connections in a dict of Channels
            :param conn_list: List of connections : Each different connection name will create a new Channel
            :param chan_dict: Preexisting channel dict. if merge wanted.
            :return:
            """
            chan_dict = chan_dict or {}
            for c in conn_list:
                if c.name not in chan_dict.keys():
                    chan_dict[c.name] = ConnectionCacheProxy.Channel(
                        c.name,
                        c.type_msg,  # type_msg is always the message type (topic or service)
                        c.type_info if c.type_info != c.type_msg else None)  # None for topics who don't have uri.
                chan_dict[c.name].nodes.add((c.node, c.xmlrpc_uri))  # type_info is the uri of the service or the msgtype of the topic
            return chan_dict

        @staticmethod
        def dict_slaughterhouse(conn_list, chan_dict):
            """
            Removes a list of Connections from a dict of Channels
            :param conn_list: List of connections : Each different connection name will affect one and only one connection
            :param chan_dict: Preexisting channel dict. to extract from
            :return:
            """
            chan_dict = chan_dict
            for c in conn_list:
                try:
                    nodelist = chan_dict[c.name].nodes
                    try:
                        nodelist.remove((c.node, c.xmlrpc_uri))
                    except KeyError:  # keep it working even if unexpected things happen
                        rospy.logwarn("Trying to remove inexistent ({c.node}, {c.xmlrpc_uri}) from connection {c.name} nodes : {nodelist} ".format(**locals()))
                        pass  # node not in set. no need to remove
                    if not nodelist:
                        chan_dict.pop(c.name)
                except KeyError:  # keep it working even if unexpected things happen
                    rospy.logwarn("Trying to access nodes for inexistent {c.name} in {chan_dict} ".format(**locals()))
                    pass  # node not in set. no need to remove

            return chan_dict

    class ActionChannel(object):
        """
        Extension of a Channel for Actions
        """
        def __init__(self, goal_chan, cancel_chan, status_chan, feedback_chan, result_chan):
            """
            Initialize an ActionChannel instance
            :param goal_chan
            :param cancel_chan
            :param status_chan
            :param feedback_chan
            :param result_chan
            :param nodes a set of tuple (node_name, node_uri)
            """
            assert goal_chan.name.endswith("/goal")
            self.goal_chan = goal_chan
            assert cancel_chan.name.endswith("/cancel")
            self.cancel_chan = cancel_chan
            assert status_chan.name.endswith("/status")
            self.status_chan = status_chan
            assert feedback_chan.name.endswith("/feedback")
            self.feedback_chan = feedback_chan
            assert result_chan.name.endswith("/result")
            self.result_chan = result_chan

        @property
        def name(self):
            return self.goal_chan.name[:-len("/goal")]

        @property
        def type(self):
            return self.goal_chan.type

        @property
        def xmlrpc_uri(self):
            return None

        @property
        def nodes(self):
            return self.goal_chan.nodes | self.cancel_chan.nodes | self.status_chan.nodes | self.feedback_chan.nodes | self.result_chan.nodes

        def __eq__(self, other):  # used for manual == operator
            if not isinstance(other, ConnectionCacheProxy.ActionChannel):
                return NotImplemented
            elif self is other:
                return True
            else:
                return (self.goal_chan == other.goal_chan and
                        self.cancel_chan == other.cancel_chan and
                        self.status_chan == other.status_chan and
                        self.feedback_chan == other.feedback_chan and
                        self.result_chan == other.result_chan)

        def is_server(self):
            return (
                # only one of them present is enough to tell
                self.goal_chan.type == SUBSCRIBER
                or self.cancel_chan.type == SUBSCRIBER
                or self.status_chan.type == PUBLISHER
                or self.feedback_chan.type == PUBLISHER
                or self.result_chan.type == PUBLISHER
            )

        def is_client(self):
            return (
                self.goal_chan.type == PUBLISHER
                or self.cancel_chan.type == PUBLISHER
                or self.status_chan.type == SUBSCRIBER
                or self.feedback_chan.type == SUBSCRIBER
                or self.result_chan.type == SUBSCRIBER
            )

        @staticmethod
        def dict_factory_actions_from_chan(chan_dict, chan_other_dict, action_dict=None):
            """
            Build ActionChannels from two list of Channels, and merge them in the dictionary.
            The behavior is symmetrical, pass pubs_list, subs_list, for action clients and the reverse for action servers
            :param chan_dict: Dict of channels
            :param chan_other_dict: symmetric dict of channels ( subscriber / publisher complement of chan_dict )
            :return:
            """
            chan_dict = chan_dict or {}
            action_dict = action_dict or {}
            new_chan_dict = chan_dict
            new_chan_other_dict = chan_other_dict

            goal_chan_from_dict = {n[:-len("/goal")]: pc for n, pc in chan_dict.iteritems() if n.endswith("/goal")}
            cancel_chan_from_dict = {n[:-len("/cancel")]: pc for n, pc in chan_dict.iteritems() if n.endswith("/cancel")}
            status_chan_from_dict = {n[:-len("/status")]: pc for n, pc in chan_other_dict.iteritems() if n.endswith("/status")}
            feedback_chan_from_dict = {n[:-len("/feedback")]: pc for n, pc in chan_other_dict.iteritems() if n.endswith("/feedback")}
            result_chan_from_dict = {n[:-len("/result")]: pc for n, pc in chan_other_dict.iteritems() if n.endswith("/result")}

            # since we need all the 5 topics anyway for an action,
            # checking only in goal dict is enough
            for k, v in goal_chan_from_dict.iteritems():
                action_name = k
                try:
                    goal_chan = goal_chan_from_dict[k]
                    cancel_chan = cancel_chan_from_dict[k]
                    status_chan = status_chan_from_dict[k]
                    feedback_chan = feedback_chan_from_dict[k]
                    result_chan = result_chan_from_dict[k]
                except KeyError:  # skip this
                    continue

                # here we should have the 5 connections
                if action_name in new_chan_dict.keys():
                    action_dict[action_name].goal_chan.nodes |= goal_chan.nodes
                    action_dict[action_name].cancel_chan.nodes |= cancel_chan.nodes
                    action_dict[action_name].status_chan.nodes |= status_chan.nodes
                    action_dict[action_name].feedback_chan.nodes |= feedback_chan.nodes
                    action_dict[action_name].result_chan.nodes |= result_chan.nodes
                else:
                    action_dict[action_name] = ConnectionCacheProxy.ActionChannel(
                        goal_chan, cancel_chan, status_chan, feedback_chan, result_chan
                    )

                # purging old used stuff
                new_chan_dict.pop(goal_chan.name, None)
                new_chan_dict.pop(cancel_chan.name, None)

                new_chan_other_dict.pop(status_chan.name, None)
                new_chan_other_dict.pop(feedback_chan.name, None)
                new_chan_other_dict.pop(result_chan.name, None)

            return action_dict, new_chan_dict, new_chan_other_dict

        @staticmethod
        def dict_slaughterhouse_actions_from_chan(chan_dict, chan_other_dict, action_dict):
            """
            Destroy ActionChannels from action_lost_dict, and merge lost pubs/subs in the chan_lost_dict and chan_lost_other_dict.
            The behavior is symmetrical, pass pubs_list, subs_list, for action clients and the reverse for action servers
            :param chan_dict: Dict of channels
            :param chan_other_dict: symmetric dict of channels ( subscriber / publisher complement of chan_dict )
            :return:
            """
            chan_dict = chan_dict or {}
            action_dict = action_dict or {}
            new_chan_dict = chan_dict
            new_chan_other_dict = chan_other_dict

            goal_chan_from_dict = {n: act.goal_chan for n, act in action_dict.iteritems() if act.goal_chan.name in chan_dict.keys()}
            cancel_chan_from_dict = {n: act.cancel_chan for n, act in action_dict.iteritems() if act.cancel_chan.name in chan_dict.keys()}
            status_chan_from_dict = {n: act.status_chan for n, act in action_dict.iteritems() if act.status_chan.name in chan_other_dict.keys()}
            feedback_chan_from_dict = {n: act.feedback_chan for n, act in action_dict.iteritems() if act.feedback_chan.name in chan_other_dict.keys()}
            result_chan_from_dict = {n: act.result_chan for n, act in action_dict.iteritems() if act.result_chan.name in chan_other_dict.keys()}

            # since we need all the 5 topics anyway for an action,
            # losing only one is enough to break the action
            to_del = []
            for k, v in action_dict.iteritems():
                action_name = k
                goal_chan = None
                cancel_chan = None
                status_chan = None
                feedback_chan = None
                result_chan = None
                # removing the matching channels from channel dict.
                # registering action as lost is enough.
                try:
                    goal_chan = goal_chan_from_dict[k]
                    chan_dict.pop(goal_chan.name)
                except KeyError:  # doesnt matter
                    pass
                try:
                    cancel_chan = cancel_chan_from_dict[k]
                    chan_dict.pop(cancel_chan.name)
                except KeyError:  # doesnt matter
                    pass
                try:
                    status_chan = status_chan_from_dict[k]
                    chan_other_dict.pop(status_chan.name)
                except KeyError:  # doesnt matter
                    pass
                try:
                    feedback_chan = feedback_chan_from_dict[k]
                    chan_other_dict.pop(feedback_chan.name)
                except KeyError:  # doesnt matter
                    pass
                try:
                    result_chan = result_chan_from_dict[k]
                    chan_other_dict.pop(result_chan.name)
                except KeyError:  # doesnt matter
                    pass

                if (goal_chan is not None or cancel_chan is not None) or (
                    status_chan is not None or feedback_chan is not None or result_chan is not None
                ):
                    to_del.append(action_name)

            # purging lost actions
            for action_name in to_del:
                action_dict.pop(action_name)

            return action_dict, new_chan_dict, new_chan_other_dict

    def __init__(self, list_sub=None, handle_actions=False, user_callback=None, diff_opt=False, diff_sub=None, list_wait_timeout=5):
        """
        Initialize a connection cache proxy to retrieve system state while minimizing call to the master or the cache node
        This method will block until the connection cache node has sent its system state
        :param list_sub: topic name to subscribe to to get the list from connectioncache node
        :param handle_actions: whether connectioncacheProxy does action filtering internally
        :param user_callback: user callback function for asynchronous state change management
        :param diff_opt: whether we optimise the proxy by using the differences only and rebuilding the full state from it
        :param diff_sub: topic name to subscribe to to get the diff from connectioncache node
        :param list_wait_timeout: seconds to wait for list in list topic before timing out
        :return:
        """
        self.diff_opt = diff_opt
        self.diff_sub = diff_sub or '~connections_diff'
        self.handle_actions = handle_actions
        self.user_cb = None
        self._system_state_lock = threading.Lock()  # writer lock
        if self.handle_actions:
            self.SystemState = collections.namedtuple("SystemState", "publishers subscribers services action_servers action_clients")
        else:
            self.SystemState = collections.namedtuple("SystemState", "publishers subscribers services")
        self._system_state = None

        if user_callback:
            if not hasattr(user_callback, '__call__'):
                rospy.logwarn("Connection Cache Proxy user callback not callable. Ignoring user callback.")
            else:
                self.user_cb = user_callback

        self.conn_list_called = threading.Event()
        self.conn_list = rospy.Subscriber(list_sub or '~connections_list', rocon_std_msgs.ConnectionsList, self._list_cb)
        if not self.conn_list_called.wait(list_wait_timeout):  # we block until we receive a message from connection node
            # if we timeout we except to prevent using the object uninitialized
            raise ConnectionCacheProxy.InitializationTimeout("Connection Cache Proxy timed out on initialization. aborting")

        # waiting until we are sure we are plugged in connection cache node.
        # RAII : after __init__() ConnectionCache is ready to use (self._system_state is initialized).

        rospy.loginfo("ConnectionCacheProxy: started inside node {}".format(rospy.get_name()))
        rospy.loginfo("                    : with list topic at {}".format(self.conn_list.name))
        rospy.loginfo("                    : and diff topic at {}".format(rospy.resolve_name(self.diff_sub)))

    @staticmethod
    def _is_topic_node_in_list(topic, node, topic_node_list):
        # TODO : there is probably a oneliner equivalent for this
        # check if cancel available
        available = False
        for candidate in topic_node_list:
            if candidate[0] == topic and node in candidate[1]:
                available = True
                break
        return available

    def _list_cb(self, data):
        self._system_state_lock.acquire()
        # we got a new full list : reset the local value for _system_state
        pubs = [c for c in data.connections if c.type == c.PUBLISHER]
        subs = [c for c in data.connections if c.type == c.SUBSCRIBER]
        svcs = [c for c in data.connections if c.type == c.SERVICE]

        pub_chans = ConnectionCacheProxy.Channel.dict_factory(pubs)
        sub_chans = ConnectionCacheProxy.Channel.dict_factory(subs)
        svc_chans = ConnectionCacheProxy.Channel.dict_factory(svcs)

        if self.handle_actions:
            action_server_chans, unused_subs_chans, pub_chans = ConnectionCacheProxy.ActionChannel.dict_factory_actions_from_chan(sub_chans, pub_chans)
            action_client_chans, pub_chans, unused_subs_chans = ConnectionCacheProxy.ActionChannel.dict_factory_actions_from_chan(pub_chans, sub_chans)
            self._system_state = self.SystemState(pub_chans, sub_chans, svc_chans, action_server_chans, action_client_chans)
        else:
            self._system_state = self.SystemState(pub_chans, sub_chans, svc_chans)

        self._system_state_lock.release()
        # rospy.loginfo("CACHE PROXY LIST_CB PUBLISHERS : {pubs}".format(pubs=self._system_state.publishers))
        # rospy.loginfo("CACHE PROXY LIST_CB SUBSCRIBERS : {subs}".format(subs=self._system_state.subscribers))
        # rospy.loginfo("CACHE PROXY LIST_CB SERVICES : {svcs}".format(svcs=self._system_state.services))

        if self.user_cb is not None and hasattr(self.user_cb, '__call__'):
            try:
                self.user_cb(self._system_state, None, None)
            except Exception as user_exc:
                rospy.logerr("Connection Cache Proxy : Diff Callback Exception {0}".format(user_exc))

        self.conn_list_called.set()  # signaling that the list_callback has been called ( for __init__ )

        if self.diff_opt:
            # hooking up to the diff and unhooking from the list
            self.conn_diff = rospy.Subscriber(self.diff_sub, rocon_std_msgs.ConnectionsDiff, self._diff_cb)
            self.conn_list.unregister()

    def _diff_cb(self, data):  # This should only run when we want to have the diff message optimization
        # modifying the system_state ( like the one provided by ROS master)
        self._system_state_lock.acquire()

        # we got a new full list : reset the local value for _system_state
        pubs_added = [c for c in data.added if c.type == c.PUBLISHER]
        subs_added = [c for c in data.added if c.type == c.SUBSCRIBER]
        svcs_added = [c for c in data.added if c.type == c.SERVICE]

        pubs_lost = [c for c in data.lost if c.type == c.PUBLISHER]
        subs_lost = [c for c in data.lost if c.type == c.SUBSCRIBER]
        svcs_lost = [c for c in data.lost if c.type == c.SERVICE]

        added_pub_chans = ConnectionCacheProxy.Channel.dict_factory(pubs_added)
        added_sub_chans = ConnectionCacheProxy.Channel.dict_factory(subs_added)
        added_svc_chans = ConnectionCacheProxy.Channel.dict_factory(svcs_added)

        lost_pub_chans = ConnectionCacheProxy.Channel.dict_factory(pubs_lost)
        lost_sub_chans = ConnectionCacheProxy.Channel.dict_factory(subs_lost)
        lost_svc_chans = ConnectionCacheProxy.Channel.dict_factory(svcs_lost)

        # we need to copy the system_state lists to be able to get meaningful diff afterwards
        old_system_state = copy.deepcopy(self._system_state)

        # new system state for services not going to change for actions
        svc_chans = ConnectionCacheProxy.Channel.dict_slaughterhouse(svcs_lost, ConnectionCacheProxy.Channel.dict_factory(svcs_added, old_system_state.services))

        if self.handle_actions:
            # changing new pubs/subs chan to new actionchans
            new_action_servers, added_sub_chans, added_pub_chans =\
                ConnectionCacheProxy.ActionChannel.dict_factory_actions_from_chan(
                    added_sub_chans, added_pub_chans, old_system_state.action_servers
                )

            new_action_clients, added_pub_chans, added_sub_chans =\
                ConnectionCacheProxy.ActionChannel.dict_factory_actions_from_chan(
                    added_pub_chans, added_sub_chans, old_system_state.action_clients
                )

            # changing lost pubs/subs chans to lost actionchans
            new_action_servers, lost_sub_chans, lost_pub_chans =\
                ConnectionCacheProxy.ActionChannel.dict_slaughterhouse_actions_from_chan(
                    lost_sub_chans, lost_pub_chans, new_action_servers
                )

            new_action_clients, lost_pub_chans, lost_sub_chans =\
                ConnectionCacheProxy.ActionChannel.dict_slaughterhouse_actions_from_chan(
                    lost_pub_chans, lost_sub_chans, new_action_clients
                )

            # recalculating difference after actions
            _added_system_state = self.SystemState(
                added_pub_chans,
                added_sub_chans,
                added_svc_chans,
                {n: c for n, c in new_action_servers.iteritems() if not (n in self._system_state.action_servers.keys() and self._system_state.action_servers[n] == c)},
                {n: c for n, c in new_action_clients.iteritems() if not (n in self._system_state.action_clients.keys() and self._system_state.action_clients[n] == c)},
            )

            _lost_system_state = self.SystemState(
                lost_pub_chans,
                lost_sub_chans,
                lost_svc_chans,
                {n: c for n, c in self._system_state.action_servers.iteritems() if not (n in new_action_servers.keys() and new_action_servers[n] == c)},
                {n: c for n, c in self._system_state.action_clients.iteritems() if not (n in new_action_clients.keys() and new_action_clients[n] == c)},
            )

            pub_chans = old_system_state.publishers
            pub_chans.update(added_pub_chans)
            pub_chans = {k: v for k, v in pub_chans.iteritems() if k not in lost_pub_chans.keys()}

            sub_chans = old_system_state.subscribers
            sub_chans.update(added_sub_chans)
            sub_chans = {k: v for k, v in sub_chans.iteritems() if k not in lost_sub_chans.keys()}

            self._system_state = self.SystemState(
                pub_chans,
                sub_chans,
                svc_chans,
                new_action_servers,
                new_action_clients,
            )

        else:
            pub_chans = ConnectionCacheProxy.Channel.dict_slaughterhouse(pubs_lost, ConnectionCacheProxy.Channel.dict_factory(pubs_added, old_system_state.publishers))
            sub_chans = ConnectionCacheProxy.Channel.dict_slaughterhouse(subs_lost, ConnectionCacheProxy.Channel.dict_factory(subs_added, old_system_state.subscribers))

            _added_system_state = self.SystemState(added_pub_chans, added_sub_chans, added_svc_chans)
            _lost_system_state = self.SystemState(lost_pub_chans, lost_sub_chans, lost_svc_chans)
            self._system_state = self.SystemState(pub_chans, sub_chans, svc_chans)

        self._system_state_lock.release()
        # rospy.loginfo("CACHE PROXY LIST_CB PUBLISHERS : {pubs}".format(pubs=self._system_state.publishers))
        # rospy.loginfo("CACHE PROXY LIST_CB SUBSCRIBERS : {subs}".format(subs=self._system_state.subscribers))
        # rospy.loginfo("CACHE PROXY LIST_CB SERVICES : {svcs}".format(svcs=self._system_state.services))

        if self.user_cb is not None and hasattr(self.user_cb, '__call__'):
            try:
                self.user_cb(self._system_state, _added_system_state, _lost_system_state)
            except Exception as user_exc:
                rospy.logerr("Connection Cache Proxy : Diff Callback Exception {0}".format(user_exc))

        pass

    def getSystemState(self):
        # ROSmaster system_state format
        self._system_state_lock.acquire()  # block in case we re changing it at the moment
        rosmaster_ss = (
            [[name, [n[0] for n in self._system_state.publishers[name].nodes]] for name in self._system_state.publishers],
            [[name, [n[0] for n in self._system_state.subscribers[name].nodes]] for name in self._system_state.subscribers],
            [[name, [n[0] for n in self._system_state.services[name].nodes]] for name in self._system_state.services],
        )
        self._system_state_lock.release()
        return rosmaster_ss

    def getTopicTypes(self):
        # ROSmaster system_state format
        self._system_state_lock.acquire()  # block in case we re changing it at the moment
        # building set of tuples to enforce unicity
        pubset = {(name, chan.type) for name, chan in self._system_state.publishers.iteritems()}
        subset = {(name, chan.type) for name, chan in self._system_state.subscribers.iteritems()}
        rosmaster_tt = [list(t) for t in (pubset | subset)]
        self._system_state_lock.release()
        return rosmaster_tt

    # Completing Master API
    def getServiceTypes(self):
        # ROSmaster system_state format
        self._system_state_lock.acquire()  # block in case we re changing it at the moment
        # building set of tuples to enforce unicity
        svcset = {(name, chan.type) for name, chan in self._system_state.services.iteritems()}
        rosmaster_st = [list(t) for t in svcset]
        self._system_state_lock.release()
        return rosmaster_st

    # Completing Master API
    def getServiceUris(self):
        # ROSmaster system_state format
        self._system_state_lock.acquire()  # block in case we re changing it at the moment
        # building set of tuples to enforce unicity
        svcset = {(name, chan.xmlrpc_uri) for name, chan in self._system_state.services.iteritems()}
        rosmaster_su = [list(t) for t in svcset]
        self._system_state_lock.release()
        return rosmaster_su
