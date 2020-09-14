#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises, assert_false
import rocon_interactions
import rocon_interactions.utils
import rocon_console.console as console

##############################################################################
# Utility functions
##############################################################################

def assign_namespace(namespace, interactions):
    for interaction in interactions:
        interaction.namespace = namespace
    return interactions

def load_interactions(resource_name):
    try:
        (msg_pairings, msg_interactions) = rocon_interactions.utils.load_msgs_from_yaml_resource(resource_name)  # interaction_msgs.Interaction[]
    except rocon_interactions.MalformedInteractionsYaml:
        msg_pairings = None
        msg_interactions = None
    return (msg_pairings, msg_interactions)

##############################################################################
# Tests
##############################################################################

def test_qt_apps():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Direct, RosRun and Launchable Interactions" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")

    interactions_table = rocon_interactions.InteractionsTable()
    raised_exception = False
    (unused_msg_pairings, msg_interactions) = load_interactions('rocon_interactions/pc')
    assert msg_interactions is not None, 'malformed yaml [rocon_interactions/pc]'
    msg_interactions = assign_namespace('/pc', msg_interactions)
    interactions_table.load(msg_interactions)
    print("%s" % interactions_table)
    assert 'PC' in interactions_table.groups()

 
def test_android_apps():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Android Interactions" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
 
    interactions_table = rocon_interactions.InteractionsTable()
    (unused_msg_pairings, msg_interactions) = load_interactions('rocon_interactions/android')
    assert msg_interactions is not None, 'malformed yaml [rocon_interactions/android]'
    msg_interactions = assign_namespace('/android', msg_interactions)
    interactions_table.load(msg_interactions)
    print("%s" % interactions_table)
    assert 'Android' in interactions_table.groups()

 
def test_web_urls():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Web Urls" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
 
    interactions_table = rocon_interactions.InteractionsTable()
    (unused_msg_pairings, msg_interactions) = load_interactions('rocon_interactions/web')
    assert msg_interactions is not None, 'malformed yaml [rocon_interactions/web]'
    msg_interactions = assign_namespace('/web', msg_interactions)
    interactions_table.load(msg_interactions)
    print("%s" % interactions_table)
    assert 'Web' in interactions_table.groups()

# This needs to move to a rostest to make use of the ros param server to do dynamic bindings
# def test_web_apps():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* Web App Interactions" + console.reset)
#     print(console.bold + "****************************************************************************************" + console.reset)
#     print("")
# 
#     interactions_table = rocon_interactions.InteractionsTable()
#     msg_interactions = load_interactions('rocon_interactions/web_apps')
#     assert msg_interactions is not None, 'malformed yaml [rocon_interactions/web_apps]'
#     interactions_table.load(msg_interactions)
#     print("%s" % interactions_table)
#     assert 'Web Apps' in interactions_table.groups()

def test_removal():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Removal" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print("")
 
    interactions_table = rocon_interactions.InteractionsTable()
    (unused_msg_pairings, msg_interactions) = load_interactions('rocon_interactions/android')
    assert msg_interactions is not None, 'malformed yaml [rocon_interactions/android]'
    msg_interactions = assign_namespace('/android', msg_interactions)
    interactions_table.load(msg_interactions)
    print("%s" % interactions_table)
    interactions_table.unload(msg_interactions)
    print("%s" % interactions_table)
    assert len(interactions_table) == 0, "did not entirely remove all interactions"

# TODO parameter tests, remapping tests, subsitution tests
