## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

"""Automatic testing for qicore file's parser
.. module:: test
"""

import os

import converter.xar_parser as xar_parser

import converter.behavior_parser as behavior_parser
import converter.behavior_sequence_parser as sequence_parser
import converter.box_interface_parser as interface_parser
import converter.animation_parser as animation_parser
import converter.flow_diagram_parser as diagram_parser


def test_parseOldFormat(parse_args):
    """ code to parse behavior.xar and get root
        looking for no throw and root not None
    """
    filename = os.path.abspath(parse_args[0])
    root_node = xar_parser.generate_tree_from_filename(filename)
    assert root_node


def test_parseNewBehaviorFormat(parse_args):
    """ code to parse behavior.xar and get root
        looking for no throw and root not None
    """
    filename = os.path.abspath(parse_args[1])
    root_node = behavior_parser.generate_tree_from_filename(filename)
    assert root_node


def test_parseNewBoxInterfaceFormat(parse_args):
    """ code to parse <box interface>.xml and get root
        looking for no throw and root not None
    """
    filename = os.path.abspath(parse_args[2])
    root_node = interface_parser.generate_tree_from_filename(filename)
    assert root_node


def test_parseNewBehaviorSequenceFormat(parse_args):
    """ code to parse <behavior sequence>.bhs and get root
        looking for no throw and root not None
    """
    filename = os.path.abspath(parse_args[3])
    root_node = sequence_parser.generate_tree_from_filename(filename)
    assert root_node


def test_parseNewAnimationFormat(parse_args):
    """ code to parse <animation>.anim and get root
        looking for no throw and root not None
    """
    filename = os.path.abspath(parse_args[4])
    root_node = animation_parser.generate_tree_from_filename(filename)
    assert root_node


def test_parseNewFlowDiagramFormat(parse_args):
    """ code to parse <flow diagram>.fld and get root
        looking for no throw and root not None
    """
    filename = os.path.abspath(parse_args[5])
    root_node = diagram_parser.generate_tree_from_filename(filename)
    assert root_node
