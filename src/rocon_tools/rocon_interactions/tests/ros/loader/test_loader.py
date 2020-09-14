#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# RosTest
##############################################################################

""" Test loading of interactions to the interactions manager. """

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import time
import unittest
import rostest
import rosunit
import rospy
import rocon_console.console as console
import rocon_interaction_msgs.srv as interaction_srvs
import rocon_uri
import rocon_interactions

##############################################################################
# Imports
##############################################################################

class TestLoader(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_loader")

    def test_loader(self):
        """ Loading... """
        try:
            rospy.wait_for_service('~get_interactions', 3.0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            self.fail("Failed to find %s" % rospy.resolve_name('~get_interactions'))
        get_pairings = rospy.ServiceProxy('~get_pairings', interaction_srvs.GetPairings)
        get_interactions = rospy.ServiceProxy('~get_interactions', interaction_srvs.GetInteractions)
        pairings_request = interaction_srvs.GetPairingsRequest()
        interactions_request = interaction_srvs.GetInteractionsRequest(groups=[], uri=rocon_uri.default_uri_string)
        interactions_table = None
        timeout_time = time.time() + 5.0
        while not rospy.is_shutdown() and time.time() < timeout_time:
            pairings_response = get_pairings(pairings_request)
            interactions_response = get_interactions(interactions_request)
            if interactions_response.interactions and pairings_response.pairings:
                pairings_table = rocon_interactions.PairingsTable()
                interactions_table = rocon_interactions.InteractionsTable()
                pairings_table.load(pairings_response.pairings)
                interactions_table.load(interactions_response.interactions)
                #print("Length: %s" % len(interactions_table))
                if len(interactions_table) == 2 and len(pairings_table) == 2:
                    break
            else:
                rospy.rostime.wallsleep(0.1)
        print("\n%s" % interactions_table)
        print("\n%s" % pairings_table)
        groups = interactions_table.groups()
        self.assertEqual(groups, ['Pairing'], 'groups of the interaction table did not return as expected [%s][%s]' % (groups, ['Rqt', 'PyQt']))
        self.assertEqual(len(interactions_table), 2, 'number of interactions incorrect [%s][%s]' % (len(interactions_table), 2))
        self.assertEqual(len(pairings_table), 2, 'number of pairings incorrect [%s][%s]' % (len(pairings_table), 2))

    def tearDown(self):
        pass

if __name__ == '__main__':
    rostest.rosrun('rocon_interactions', 'loader', TestLoader)
