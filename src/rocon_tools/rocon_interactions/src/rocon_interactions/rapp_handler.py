#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Module
##############################################################################

"""
.. module:: rapp_handler
   :platform: Unix
   :synopsis: Works with the rocon app manager to support pairing interactions.

This module provides a class that can be used by other nodes to introspect
and start/stop rapps on a rapp manager running on the same ros master.

"""

##############################################################################
# Imports
##############################################################################

import rospy
import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs
import rocon_python_comms
from .exceptions import(FailedToStartRappError,
                        FailedToStopRappError,
                        RappNotRunningError
                        )

##############################################################################
# Utilities
##############################################################################


def rapp_msg_to_dict(rapp):
    """
    Convert rapp message to a dictionary. This is not really necessary and
    we'd be better off flinging around the actual msg or a wrapper around
    the msg.
    """
    dict_rapp = {}
    dict_rapp["status"] = rapp.status
    dict_rapp["name"] = rapp.name
    dict_rapp["display_name"] = rapp.display_name
    dict_rapp["description"] = rapp.description
    dict_rapp["compatibility"] = rapp.compatibility
    dict_rapp["preferred"] = rapp.preferred
    dict_rapp["icon"] = rapp.icon
    dict_rapp["implementations"] = rapp.implementations
    dict_rapp["public_interface"] = rapp.public_interface
    dict_rapp["public_parameters"] = rapp.public_parameters
    return dict_rapp


def rapp_list_msg_to_dict(list_rapp):
    dict_rapp = {}
    for rapp in list_rapp:
        name = rapp.name
        dict_rapp[name] = rapp_msg_to_dict(rapp)
    return dict_rapp

##############################################################################
# Classes
##############################################################################


class RappHandler(object):
    """
    Initialises from a conductor message detailing information about a
    concert client. Once established, this instance can be used as
    a convenience to start and stop rapps on the concert client.

    :ivar is_running: flag indicating if there is a monitored rapp running on the rapp manager.
    :vartype is_running: bool
    :ivar _running_rapp: None, or dict representation of the running rapp (result of :py:class:`rapp_msg_to_dict()<rocon_interactions.rapp_handler.rapp_msg_to_dict>`)
    :vartype _running_rapp: dict
    """
    def __init__(self, rapp_running_state_changed_callback):
        """
        Initialise the class with the relevant data required to start and stop
        rapps on this concert client.
        """
        self._running_rapp = None
        self._available_rapps = {}
        self.rapp_running_state_changed_callback = rapp_running_state_changed_callback
        self.subscribers = rocon_python_comms.utils.Subscribers(
            [
                ('~status', rocon_app_manager_msgs.Status, self._status_subscriber_callback),
            ]
        )
        self.service_proxies = rocon_python_comms.utils.ServiceProxies(
            [
                ('~start_rapp', rocon_app_manager_srvs.StartRapp),
                ('~stop_rapp', rocon_app_manager_srvs.StopRapp),
            ]
        )
        # self._initialising_thread = threading.Thread(target=self.initialise)
        # self._initialising_thread.start()
        # this is blocking until it gets data
        self.initialise()

    @property
    def available_rapps(self):
        return self._available_rapps

    @property
    def initialised(self):
        return len(self._available_rapps) != 0

    @property
    def is_running(self):
        return self._running_rapp is not None

    @property
    def running_rapp(self):
        return self._running_rapp

    def matches_running_rapp(self, rapp_name, remappings, parameters):
        '''
        Compare the running rapp with an interaction's specification for a paired rapp.

        .. todo:: currently it checks against the rapp name only, expand this to consider remappings and parameters

        :param str rapp_name: ros package resource name for this interaction's rapp rapp
        :param rocon_std_msgs/Remapping[] remappings: rapp remapping rules for this interaction
        :param rocon_std_msgs/KeyValue[] parameters: rapp parameter rules for this interaction
        :returns: whether the incoming interaction specification matches the running rapp (used for filtering)
        :rtype; bool

        :raises: :exc:`rocon_interactions.exceptions.RappNotRunningError` if the rapp wasn't running
        '''
        try:
            if self._running_rapp['name'] == rapp_name:
                return True
        except TypeError:  # NoneType
            raise RappNotRunningError
        return False

    def is_available_rapp(self, rapp_name):
        return True if rapp_name in self._available_rapps.keys() else False

    def get_rapp(self, rapp_name):
        try:
            return self._available_rapps[rapp_name]
        except KeyError:
            return None

    def start(self, rapp, remappings, parameters=[]):
        """
        Start the rapp with the specified remappings.

        :param str rapp: ros package resource name of the rapp to start (e.g. rocon_apps/teleop)
        :param remappings: remappings to apply to the rapp when starting.
        :type remappings: [rocon_std_msgs.Remapping]
        :param parameters: paramters to apply to the rapp when starting.
        :type remappings: [rocon_std_msgs.KeyValue]

        .. include:: weblinks.rst

        :raises: :exc:`.FailedToStartRappError`
        """
        if not self.initialised:
            raise FailedToStartRappError("rapp manager's location unknown")
        try:
            unused_response = self.service_proxies.start_rapp(rocon_app_manager_srvs.StartRappRequest(name=rapp, remappings=remappings, parameters=parameters))
            # todo check this response and process it
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStartRappError("%s" % str(e))

    def stop(self):
        """
        Stop a rapp on this concert client (if one should be running). This
        doesn't need a rapp specification since only one rapp can ever be
        running - it will just stop the currently running rapp.

        :raises: :exc:`.FailedToStopRappError`
        """
        if not self.initialised:
            raise FailedToStopRappError("rapp manager's location not known")
        try:
            unused_response = self.service_proxies.stop_rapp(rocon_app_manager_srvs.StopRappRequest())
        except (rospy.service.ServiceException,
                rospy.exceptions.ROSInterruptException) as e:
            # Service not found or ros is shutting down
            raise FailedToStopRappError("%s" % str(e))

    def initialise(self):
        """
        Loops around (indefinitely) until it makes a connection with the rapp manager and retrieves the rapp list.
        """
        # get the rapp list - just loop around until catch it once - it is not dynamically changing
        rospy.loginfo("Interactions : calling the rapp manager to get the rapp list.")
        get_rapp_list = rocon_python_comms.SubscriberProxy('~rapp_list', rocon_app_manager_msgs.RappList)
        while not rospy.is_shutdown():
            # msg is rocon_app_manager_msgs/RappList
            # this returns almost immediately with None if rospy gets shutdown, so a long duration is ok
            msg = get_rapp_list(rospy.Duration(30.0))
            if msg is None:
                rospy.logwarn("Interactions : unable to connect with the rapp manager : {0} not found, will keep trying.".format(rospy.resolve_name('~rapp_list')))
            else:
                self._available_rapps = rapp_list_msg_to_dict(msg.available_rapps)
                rospy.loginfo("Interactions : discovered rapp support for pairing modes:")
                for rapp in msg.available_rapps:
                    rospy.loginfo("Interactions :     '%s'" % rapp.name)
                break
        get_rapp_list.unregister()

    def _status_subscriber_callback(self, msg):
        """
        Update the current status of the rapp manager
        @param rocon_app_manager_msgs/Status msg: detailed report of the rapp manager's current state
        """
        state_changed = False
        stopped = False
        if msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_RUNNING:
            # FIXME : This should probably be done internally in the app_manager
            # => A we only need the published interface from a running app, without caring about the original specification
            # => Is this statement always true ?
            running_rapp = rapp_msg_to_dict(msg.rapp)
            if self._running_rapp is None or self._running_rapp['name'] != running_rapp['name']:
                state_changed = True
#             for pubif_idx, pubif in enumerate(running_rapp['public_interface']):
#                 newvals = ast.literal_eval(pubif.value)
#                 for msgif in msg.published_interfaces:
#                     if (
#                         (pubif.key == 'subscribers' and msgif.interface.connection_type == 'subscriber') or
#                         (pubif.key == 'publishers' and msgif.interface.connection_type == 'publisher')
#                     ):
#                         # rospy.loginfo('newvals %r', newvals)
#                         for newval_idx, newval in enumerate(newvals):
#                             # rospy.loginfo('newvals[%r] %r', newval_idx, newval)
#                             if newval['name'] == msgif.interface.name and newval['type'] == msgif.interface.data_type:
#                                 # Careful we re changing the list in place here
#                                 newvals[newval_idx]['name'] = msgif.name
#                                 # rospy.loginfo('newvals[%r] -> %r', newval_idx, newval)
#                                 # Careful we re changing the list in place here
#                     elif msgif.interface.connection_type not in rocon_python_comms.connections.connection_types:
#                         rospy.logerr('Interactions : unsupported connection type : %r', msgif.interface.connection_type)
#                 # Careful we re changing the list in place here
#                 running_rapp['public_interface'][pubif_idx].value = str(newvals)
#             TODO : same for published parameters ?
            self._running_rapp = running_rapp
            # rospy.loginfo('Interactions : new public if : %r', self._running_rapp['public_interface'])
        elif msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_STOPPED:
            was_running = self._running_rapp is not None
            self._running_rapp = None
            if was_running:
                state_changed = True
                stopped = True  # signal the higher level disable pairing mode via this
        if state_changed:
            self.rapp_running_state_changed_callback(self._running_rapp, stopped)
