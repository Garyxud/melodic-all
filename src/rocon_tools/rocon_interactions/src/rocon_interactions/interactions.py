#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: interactions
   :platform: Unix
   :synopsis: Representative class and methods for an *interaction*.


This module defines a class and methods that represent the core of what
an interaction is.

----

"""
##############################################################################
# Imports
##############################################################################

import rospkg
import rocon_console.console as console
import rocon_python_utils

from .exceptions import InvalidInteraction
from . import web_interactions
from . import utils

##############################################################################
# Classes
##############################################################################


class Interaction(object):
    '''
      This class defines an interaction. It does so by wrapping the base
      rocon_interaction_msgs.Interaction_ msg structure with
      a few convenient variables and methods.

      .. include:: weblinks.rst
    '''
    __slots__ = [
        'msg',           # rocon_interaction_msgs.Interaction
    ]

    def __init__(self, msg):
        """
          Validate the incoming fields supplied by the interaction msg
          and populate remaining fields with proper defaults (e.g. calculate the
          unique hash for this interaction). The hash is calculated based on the
          incoming name-group-namespace triple.

          :param msg: underlying data structure with fields minimally filled via :func:`.load_msgs_from_yaml_resource`.
          :type msg: rocon_interaction_msgs.Interaction_

          :raises: :exc:`.InvalidInteraction` if the interaction variables were improperly defined (e.g. max = -1)

          .. include:: weblinks.rst
        """
        self.msg = msg
        """Underlying data structure (rocon_interaction_msgs.Interaction_)"""
        if self.msg.max < -1:
            raise InvalidInteraction("maximum instance configuration cannot be negative [%s]" % self.msg.name)
        if self.msg.max == 0:
            self.msg.max = 1
        if self.msg.group == '':
            raise InvalidInteraction("group not configured [%s]" % self.msg.name)
        if self.msg.icon.resource_name == "":
            self.msg.icon.resource_name = 'rocon_bubble_icons/rocon.png'
        if not self.msg.icon.data:
            try:
                self.msg.icon = rocon_python_utils.ros.icon_resource_to_msg(self.msg.icon.resource_name)
            except rospkg.common.ResourceNotFound as unused_e:  # replace with default icon if icon resource is not found.
                self.msg.icon.resource_name = 'rocon_bubble_icons/rocon.png'
                self.msg.icon = rocon_python_utils.ros.icon_resource_to_msg(self.msg.icon.resource_name)
        if self.msg.namespace == '':
            self.msg.namespace = '/'
        self.msg.hash = utils.generate_hash(self.msg.name, self.msg.group, self.msg.namespace)
        # some convenient aliases - these should be properties!

    def is_paired_type(self):
        """
        Classify whether this interaction is to be paired with a rapp or not.

        :returns: whether it is a pairing interaction or not
        :rtype: bool
        """
        return True if self.msg.required_pairings else False

    ##############################################################################
    # Conveniences
    ##############################################################################

    @property
    def command(self):
        """Executable name for this interaction, can be a roslaunch, rosrunnable, global executable, web url or web app [int]."""
        return self.msg.command

    @property
    def group(self):
        """The group under which this interaction should be embedded [int]."""
        return self.msg.group

    @property
    def compatibility(self):
        """A rocon_uri_ string that indicates what platforms it may run on [int]."""
        return self.msg.compatibility

    @property
    def name(self):
        """A human friendly name that also uniquely helps uniquely identify this interaction (you can have more than one configured ``command`` instance) [int]."""
        return self.msg.name

    @property
    def description(self):
        return self.msg.description

    @property
    def icon(self):
        return self.msg.icon

    @property
    def namespace(self):
        """Default namespace under which ros services and topics should be embedded for this interaction [int]."""
        return self.msg.namespace

    @property
    def max(self):
        """
        Maximum number of instantiations that is permitted (e.g. teleop should only allow 1) [int].
        """
        return self.msg.max

    @property
    def remappings(self):
        return self.msg.remappings

    @property
    def parameters(self):
        return self.msg.parameters

    @property
    def hash(self):
        """A crc32 unique identifier key for this interaction, see also :func:`.generate_hash` [int32]."""
        return self.msg.hash

    @property
    def bringup_pairing(self):
        return self.msg.bringup_pairing

    @property
    def teardown_pairing(self):
        return self.msg.teardown_pairing

    @property
    def required_pairings(self):
        return self.msg.required_pairings

    def __eq__(self, other):
        if type(other) is type(self):
            return self.msg.hash == other.msg.hash
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        '''
          Format the interaction into a human-readable string.
        '''
        web_interaction = web_interactions.parse(self.msg.command)
        command = self.msg.command if web_interaction is None else web_interaction.url
        s = ''
        s += console.green + "%s" % self.msg.name + console.reset + '\n'
        s += console.cyan + "  Command" + console.reset + "          : " + console.yellow + "%s" % command + console.reset + '\n'  # noqa
        s += console.cyan + "  Description" + console.reset + "      : " + console.yellow + "%s" % self.msg.description + console.reset + '\n'  # noqa
        s += console.cyan + "  Icon" + console.reset + "             : " + console.yellow + "%s" % str(self.msg.icon.resource_name) + console.reset + '\n'  # noqa
        s += console.cyan + "  Rocon URI" + console.reset + "        : " + console.yellow + "%s" % self.msg.compatibility + console.reset + '\n'  # noqa
        s += console.cyan + "  Namespace" + console.reset + "        : " + console.yellow + "%s" % self.msg.namespace + console.reset + '\n'  # noqa
        if self.msg.max == -1:
            s += console.cyan + "  Max" + console.reset + "              : " + console.yellow + "infinity" + console.reset + '\n'  # noqa
        else:
            s += console.cyan + "  Max" + console.reset + "              : " + console.yellow + "%s" % self.msg.max + console.reset + '\n'  # noqa
        already_prefixed = False
        for remapping in self.msg.remappings:
            if not already_prefixed:
                s += console.cyan + "  Remappings" + console.reset + "       : " + console.yellow + "%s->%s" % (remapping.remap_from, remapping.remap_to) + console.reset + '\n'  # noqa
                already_prefixed = True
            else:
                s += "                   : " + console.yellow + "%s->%s" % (remapping.remap_from, remapping.remap_to) + console.reset + '\n'  # noqa
        if self.msg.parameters != '':
            s += console.cyan + "  Parameters" + console.reset + "       : " + console.yellow + "%s" % self.msg.parameters + console.reset + '\n'  # noqa
        if self.msg.required_pairings:
            s += console.cyan + "  Bringup Pairing " + console.reset + " : " + console.yellow + "%s" % self.msg.bringup_pairing + console.reset + '\n'  # noqa
            s += console.cyan + "  Teardown Pairing " + console.reset + ": " + console.yellow + "%s" % self.msg.teardown_pairing + console.reset + '\n'  # noqa
            s += console.cyan + "  Required Pairings" + console.reset + ": " + console.yellow + "%s" % self.msg.required_pairings + console.reset + '\n'  # noqa
        s += console.cyan + "  Hash" + console.reset + "             : " + console.yellow + "%s" % str(self.msg.hash) + console.reset + '\n'  # noqa
        return s
