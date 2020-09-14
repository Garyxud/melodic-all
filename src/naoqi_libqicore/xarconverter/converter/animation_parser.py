#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import xml.sax

import converter.xar_types as xar_types
import converter.node.animation as node_animation
import converter.node.actuator_list as node_actuator_list
import converter.node.actuator_curve as node_actuator_curve
import converter.node.actuator_key as node_actuator_key


def generate_tree_from_filename(filename):
    """ Generates the representation of the file designed by filename
        in memory. Build a tree.

        :param filename: name of the file to parse
        :returns: the tree representing the file
    """
    parser = xml.sax.make_parser()
    handler = AnimationHandler()
    parser.setContentHandler(handler)
    parser.parse(open(filename))

    root_node = handler.get_root()
    if root_node.format_version != "4":
        return None

    return root_node


class AnimationHandler(xml.sax.handler.ContentHandler):
    """ ContentHandler to parse the animation file
    """

    def __init__(self):
        xml.sax.handler.ContentHandler.__init__(self)
        self._nodes = []
        self._root = None

    def startElement(self, name, attrs):
        new_node = None
        if name == 'Animation':
            new_node = node_animation.Animation(
                xar_types.attributes(attrs))
        elif name == 'ActuatorList':
            new_node = node_actuator_list.ActuatorList(
                xar_types.attributes(attrs))
        elif name == 'ActuatorCurve':
            new_node = node_actuator_curve.ActuatorCurve(
                xar_types.attributes(attrs))
        elif name == 'Key':
            new_node = node_actuator_key.ActuatorKey(
                xar_types.attributes(attrs))

        if new_node:
            if not self._root:
                self._root = new_node
            # Attach to parent here
            if self._nodes:
                parent_node = self._nodes.pop()
                parent_node.attach_attribute(name, new_node)
                parent_node.add_child(new_node)
                self._nodes.append(parent_node)

            self._nodes.append(new_node)
        else:
            raise Exception("The node %s is unknown" % name)

    def endElement(self, name):
        self._nodes.pop()

    def get_root(self):
        return self._root
