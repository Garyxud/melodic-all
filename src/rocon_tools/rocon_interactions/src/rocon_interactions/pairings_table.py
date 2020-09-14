#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: pairings_table
   :platform: Unix
   :synopsis: A database of pairings.


This module provides a class that acts as a database (dictionary style) of
some set of pairings.

----

"""
##############################################################################
# Imports
##############################################################################

import rocon_console.console as console

from . import pairings
from .exceptions import InvalidInteraction

##############################################################################
# Classes
##############################################################################


class PairingsTable(object):
    '''
      The runtime populated pairings table along with methods to
      manipulate it.

      :ivar pairings: list of objects that form the elements of the table
      :vartype pairings: rocon_interactions.pairings.Pairing[]
    '''
    def __init__(self):
        """
        Constructs an empty pairings table.
        """
        self.pairings = []

    def __len__(self):
        return len(self.pairings)

    def __str__(self):
        """
        Convenient string representation of the table.
        """
        s = console.bold + "Pairings" + console.reset + '\n'
        for pairing in self.pairings:
            s += "\n".join("  " + i for i in str(pairing).splitlines()) + '\n'
        return s

    def sorted(self):
        """
        Return the pairing list sorted by name.
        """
        return sorted(self.pairings, key=lambda pairing: pairing.name)

    def load(self, msgs):
        '''
          Load some pairings into the table. This involves some initialisation
          and validation steps.

          :param msgs: a list of interaction specifications to populate the table with.
          :type msgs: rocon_interaction_msgs.Interaction_ []
          :returns: list of all additions and any that were flagged as invalid
          :rtype: (:class:`.Pairing` [], rocon_interaction_msgs.Pairing_ []) : (new, invalid)
        '''
        new = []
        invalid = []
        for msg in msgs:
            try:
                pairing = pairings.Pairing(msg)
                self.pairings.append(pairing)
                self.pairings = list(set(self.pairings))  # uniquify the list, just in case
                new.append(pairing)
            except InvalidInteraction:
                invalid.append(msg)
        return new, invalid

    def unload(self, msgs):
        '''
          Removed the specified pairings from the table.

          :param msgs: a list of pairings
          :type msgs: rocon_interaction_msgs.Pairing_ []

          :returns: a list of removed interactions
          :rtype: rocon_interaction_msgs.Pairing_ []
        '''
        removed = []
        for msg in msgs:
            found = self.find(msg.name)
            if found is not None:
                removed.append(msg)
                self.pairings.remove(found)
        return removed

    def find(self, name):
        '''
          Find the specified pairing.

          :param str name: unique name

          :returns: pairing if found, None otherwise.
          :rtype: :class:`.Pairing`
        '''
        pairing = next((p for p in self.pairings if p.name == name), None)
        return pairing

    def is_available_pairing(self, pairing_name):
        return pairing_name in [p.name for p in self.pairings]
