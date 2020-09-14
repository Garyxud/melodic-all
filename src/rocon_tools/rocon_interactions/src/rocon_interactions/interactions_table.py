#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: interactions_table
   :platform: Unix
   :synopsis: A database of interactions.


This module provides a class that acts as a database (dictionary style) of
some set of interactions.

----

"""
##############################################################################
# Imports
##############################################################################

import re
import rocon_console.console as console
import rocon_uri
import rospy

from . import interactions
from . import utils
from .exceptions import InvalidInteraction, MalformedInteractionsYaml

##############################################################################
# Classes
##############################################################################


class InteractionsTable(object):
    '''
      The runtime populated interactions table along with methods to
      manipulate it.

      .. include:: weblinks.rst

      :ivar interactions: list of objects that form the elements of the table
      :vartype interactions: rocon_interactions.interactions.Interaction[]
      :ivar filter_pairing_interactions: flag for indicating whether pairing interactions should be filtered when loading.
      :vartype filter_pairing_interactions: bool
    '''
    def __init__(self,
                 filter_pairing_interactions=False
                 ):
        """
        Constructs an empty interactions table.

        :param bool filter_pairing_interactions: do not load any paired interactions
        """
        self.interactions = []
        self.filter_pairing_interactions = filter_pairing_interactions

    def groups(self):
        '''
          List all groups for the currently stored interactions.

          :returns: a list of all groups
          :rtype: str[]
        '''
        # uniquify the list
        return sorted(list(set([i.group for i in self.interactions])))

    def __len__(self):
        return len(self.interactions)

    def __str__(self):
        """
        Convenient string representation of the table.
        """
        s = ''
        group_view = self.generate_group_view()
        for group, interactions in group_view.iteritems():
            s += console.bold + "Interactions - " + group + console.reset + '\n'
            for interaction in interactions:
                s += "\n".join("  " + i for i in str(interaction).splitlines()) + '\n'
        return s

    def sorted(self):
        """
        Return the interactions list sorted by name.
        """
        return sorted(self.interactions, key=lambda interaction: interaction.name)

    def generate_group_view(self):
        '''
          Creates a temporary copy of the interactions and sorts them into a dictionary
          view classified by group.

          :returns: A group based view of the interactions
          :rtype: dict { group(str) : :class:`.interactions.Interaction`[] }
        '''
        # there's got to be a faster way of doing this.
        interactions = list(self.interactions)
        group_view = {}
        for interaction in interactions:
            if interaction.group not in group_view.keys():
                group_view[interaction.group] = []
            group_view[interaction.group].append(interaction)
        return group_view

    def filter(self, groups=None, compatibility_uri='rocon:/'):
        '''
          Filter the interactions in the table according to group and/or compatibility uri.

          :param groups: a list of groups to filter against, use all groups if None
          :type groups: str []
          :param str compatibility_uri: compatibility rocon_uri_, eliminates interactions that don't match this uri.

          :returns interactions: subset of all interactions that survived the filter
          :rtype: :class:`.Interaction` []

          :raises: rocon_uri.RoconURIValueError if provided compatibility_uri is invalid.
        '''
        if groups:   # works for classifying non-empty list vs either of None or empty list
            group_filtered_interactions = [i for i in self.interactions if i.group in groups]
        else:
            group_filtered_interactions = list(self.interactions)
        filtered_interactions = [i for i in group_filtered_interactions
                                 if rocon_uri.is_compatible(i.compatibility, compatibility_uri)]
        return filtered_interactions

    def load(self, msgs):
        '''
          Load some interactions into the table. This involves some initialisation
          and validation steps.

          :param msgs: a list of interaction specifications to populate the table with.
          :type msgs: rocon_interaction_msgs.Interaction_ []
          :returns: list of all additions and any that were flagged as invalid
          :rtype: (:class:`.Interaction` [], rocon_interaction_msgs.Interaction_ []) : (new, invalid)
        '''
        msgs = self._bind_dynamic_symbols(msgs)
        new = []
        invalid = []
        for msg in msgs:
            try:
                interaction = interactions.Interaction(msg)
                self.interactions.append(interaction)
                self.interactions = list(set(self.interactions))  # uniquify the list, just in case
                new.append(interaction)
            except InvalidInteraction:
                invalid.append(msg)
        return new, invalid

    def unload(self, msgs):
        '''
          Removed the specified interactions interactions table. This list is typically
          the same list as the user might initially send - no hashes yet generated.

          :param msgs: a list of interactions
          :type msgs: rocon_interaction_msgs.Interaction_ []

          :returns: a list of removed interactions
          :rtype: rocon_interaction_msgs.Interaction_ []
        '''
        removed = []
        for msg in msgs:
            msg_hash = utils.generate_hash(msg.name, msg.group, msg.namespace)
            found = self.find(msg_hash)
            if found is not None:
                removed.append(msg)
                self.interactions.remove(found)
        return removed

    def find(self, interaction_hash):
        '''
          Find the specified interaction.

          :param str interaction_hash: in crc32 format

          :returns: interaction if found, None otherwise.
          :rtype: :class:`.Interaction`
        '''
        interaction = next((interaction for interaction in self.interactions
                            if interaction.hash == interaction_hash), None)
        return interaction

    def _bind_dynamic_symbols(self, interaction_msgs):
        '''
          Provide some intelligence to the interactions specification by binding designated
          symbols at runtime. Commonly used bindings and their usage points include:

          - interaction.name - __WEBSERVER_ADDRESS__
          - interaction.parameters - __ROSBRIDGE_ADDRESS__
          - interaction.parameters - __ROSBRIDGE_PORT__

          :param interaction_msgs: parse this interaction scanning and replacing symbols.
          :type interaction_msgs: rocon_interactions_msgs.Interaction[]

          :returns: the updated interaction list
          :rtype: rocon_interactions_msgs.Interaction[]
        '''
        # search for patterns of the form '<space>__PARAMNAME__,'
        # and if found, look to see if there is a rosparam matching that pattern that we can substitute
        pattern = '\ __(.*?)__[,|\}]'
        for interaction in interaction_msgs:
            for p in re.findall(pattern, interaction.parameters):
                rparam = None
                if p.startswith('/'):
                    try:
                        rparam = rospy.get_param(p)
                    except KeyError:
                        pass  # we show a warning below
                elif p.startswith('~'):
                    msg = '%s is invalid format for rosparam binding. See https://github.com/robotics-in-concert/rocon_tools/issues/81' % p
                    raise MalformedInteractionsYaml(str(msg))
                else:
                    # See https://github.com/robotics-in-concert/rocon_tools/issues/81 for the rule
                    if interaction.namespace:
                        try:
                            rparam = rospy.get_param(interaction.namespace)
                        except KeyError:
                            pass  # fallback and try again in the private namespace
                    if rparam is None:
                        try:
                            rparam = rospy.get_param('~' + p)
                        except KeyError:
                            pass  # we show a warning below
                if rparam is None:
                    rospy.logwarn("Interactions : no dynamic binding found for '%s'" % p)
                match = '__' + p + '__'
                interaction.parameters = interaction.parameters.replace(match, str(rparam))
        return interaction_msgs
