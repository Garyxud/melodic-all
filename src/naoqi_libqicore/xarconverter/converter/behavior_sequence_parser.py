#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import xml.sax

import converter.xar_types as xar_types

import converter.node.behavior_sequence as node_behavior_sequence
import converter.node.behavior_layer as node_behavior_layer
import converter.node.behavior_keyframe_v2 as node_behavior_keyframe_v2


def generate_tree_from_filename(filename):
    """ Generates the representation of the file designed by filename
        in memory. Build a tree.

        :param filename: name of the file to parse
        :returns: the tree representing the file
    """
    parser = xml.sax.make_parser()
    handler = BehaviorSequenceHandler()
    parser.setContentHandler(handler)
    parser.parse(open(filename))

    root_node = handler.get_root()
    if root_node.format_version != "4":
        return None

    return root_node


class BehaviorSequenceHandler(xml.sax.handler.ContentHandler):
    """ ContentHandler to parse the xar file
    """

    def __init__(self):
        xml.sax.handler.ContentHandler.__init__(self)
        self._nodes = []
        self._root = None

    def startElement(self, name, attrs):
        new_node = None
        if name == 'BehaviorSequence':
            new_node = node_behavior_sequence.BehaviorSequence(
                xar_types.attributes(attrs))
        elif name == 'BehaviorLayer':
            new_node = node_behavior_layer.BehaviorLayer(
                xar_types.attributes(attrs))
        elif name == 'BehaviorKeyframe':
            new_node = node_behavior_keyframe_v2.BehaviorKeyframeV2(
                xar_types.attributes(attrs))

        if not self._root:
            self._root = new_node
        # Attach to parent here
        if self._nodes:
            parent_node = self._nodes.pop()
            parent_node.attach_attribute(name, new_node)
            parent_node.add_child(new_node)
            self._nodes.append(parent_node)

        self._nodes.append(new_node)

    def endElement(self, name):
        if (name == 'BehaviorSequence'
                or name == 'BehaviorLayer'
                or name == 'BehaviorKeyframe'):
            self._nodes.pop()

    def get_root(self):
        return self._root
