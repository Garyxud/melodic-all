#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: rules
   :platform: Unix
   :synopsis: EBNF rules used in parsing rocon_uri strings.

This module defines `ebnf rules`_ used in parsing rocon_uri strings. It should
never be used directly, it is the engine for the core parsing and
manipulation module.

.. _`ebnf rules`: http://en.wikipedia.org/wiki/Extended_Backus%E2%80%93Naur_Form

----

"""

##############################################################################
# Imports
##############################################################################

import os
import yaml

# Local imports

##############################################################################
# Rules
##############################################################################


def load_rules_into_dictionary():
    """
      Load the rules in rocon_uri/src/rocon_uri/rules/rules.yaml into a python dictionary
      object.

      :returns: python dictionary of rules loaded from yaml.
      :rtype: str
    """
    yaml_filename = os.path.join(os.path.dirname(__file__), 'rules', 'rules.yaml')
    with open(yaml_filename) as f:
        yaml_rules = yaml.load(f)
    return yaml_rules


def walk_yaml_rules(name, root=None):
    '''
    A generator which walks through the yaml list of rules.
    Works in almost exactly the same way as os.path.walk. If a root for a
    ebnf rules dictionary is not specified, it will load the default rules dictionary, see
    :func:`.load_rules_into_dictionary`.

    Usage:::

        for name, group, elements in walk_yaml('hardware_platform', yaml_rules['hardware_platform']):
            print("Name: %s" % name)
            print("  Group: %s" % group)
            print("  Elements: %s" % elements)

    :param name: a name to attach to the current 3-tuple that gets yielded by the generator.
    :type name: str

    :param root: a python dictionary representing the root of an object loaded from a yaml rules file
    :type root: dict
    '''
    #print("Walking %s" % name)
    if root is None:
        root = load_rules_into_dictionary()
    groups = {}
    elements = []
    for element in root:
        if isinstance(element, dict):
            groups.update(element)
        else:
            elements.append(element)
    # Make sure the elements are in reverse order so when ebnf rule matching happens
    # turtlebot2 tries to match before turtlebot
    #   https://github.com/robotics-in-concert/rocon_tools/issues/17
    elements.sort(reverse=True)
    #print("  Groups: %s" % groups.keys())
    #print("  Elements: %s" % elements)
    yield (name, groups.keys(), elements)
    if not groups.keys():
        return
    for key, value in groups.iteritems():
        for x in walk_yaml_rules(name + '/' + key, value):
            yield x
    return


def load_ebnf_rules():
    """
    Load our rules from yaml and construct an `ebnf <http://wiki.ros.org/rocon_ebnf>`_
    rules dictionary for parsing rocon uri strings.

    :returns: python dictionary of rules loaded from yaml.
    :rtype: str
    """
    yaml_rule_sets = {}
    yaml_rules = load_rules_into_dictionary()
    for yaml_rule_set in yaml_rules:  # merge each of hardware_platform, application_framework, os into one dictionary
        yaml_rule_sets.update(yaml_rule_set)
    # special case, add the names as an empty list
    yaml_rule_sets['name'] = []
    for yaml_rule_set_name, yaml_rule_set in yaml_rule_sets.iteritems():
        rules = []
        #rules.append('option verbose')
        rules.append('init %s_list=[]' % yaml_rule_set_name)
        rules.append('pattern ::= zero element*')
        rules.append('zero    ::= %s  @%s_list.append("$%s")' % (yaml_rule_set_name, yaml_rule_set_name, yaml_rule_set_name))
        rules.append('element ::= "|" %s   @%s_list.append("$%s")' % (yaml_rule_set_name, yaml_rule_set_name, yaml_rule_set_name))
        for name, groups, elements in walk_yaml_rules(yaml_rule_set_name, yaml_rule_set):
            # Accept a wildcard for each
            rule = '%s ::= "*"' % name.split('/')[-1]
            element_rules = ' | '.join(['"%s"' % element for element in elements])
            group_rules = ' | '.join(groups)
            if groups:
                rule += " | " + group_rules
            if elements:
                rule += " | " + element_rules
            # special case - let anything through for names.
            if yaml_rule_set_name == "name":
                rule += ' | r"\S"*'
            rules.append(rule)
        yaml_rule_sets[yaml_rule_set_name] = rules
    return yaml_rule_sets
