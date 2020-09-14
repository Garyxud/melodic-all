#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import xml.sax

import converter.xar_types as xar_types

import converter.node.choregraphe_project as node_choregraphe_project
import converter.node.box_instance as node_box_instance
import converter.node.parameter_value as node_parameter_value
import converter.node.plugin_content as node_plugin_content


def generate_tree_from_filename(filename):
    """ Generates the representation of the file designed by filename
        in memory. Build a tree.

        :param filename: name of the file to parse
        :returns: the tree representing the file
    """
    parser = xml.sax.make_parser()
    handler = BehaviorHandler()
    parser.setContentHandler(handler)
    parser.parse(open(filename))

    root_node = handler.get_root()
    if root_node.format_version != "4":
        return None

    return root_node


class BehaviorHandler(xml.sax.handler.ContentHandler):
    """ ContentHandler to parse the xar file
    """

    def __init__(self):
        xml.sax.handler.ContentHandler.__init__(self)
        self._root = None
        self._buffer = ""
        self._nodes = []

    def startElement(self, name, attrs):
        new_node = None
        if name == 'ChoregrapheProject':
            new_node = node_choregraphe_project.ChoregrapheProject(
                xar_types.attributes(attrs))
        elif name == 'BoxInstance':
            new_node = node_box_instance.BoxInstance(
                xar_types.attributes(attrs))
        elif name == 'ParameterValue':
            new_node = node_parameter_value.ParameterValue(
                xar_types.attributes(attrs))
        elif name == 'PluginContent':
            new_node = node_plugin_content.PluginContent(
                xar_types.attributes(attrs))

        if new_node:
            if not self._root:
                self._root = new_node
            if self._nodes:
                parent_node = self._nodes.pop()
                parent_node.attach_attribute(name, new_node)
                parent_node.add_child(new_node)
                self._nodes.append(parent_node)

            self._nodes.append(new_node)
        else:
            """ pluginContent beacon can embed anything, raw string as well as
                sub nodes.
            """
            new_node = node_plugin_content.PluginSubNode(name, attrs)
            parent_node = self._nodes.pop()
            parent_node.add_subnode(new_node)
            parent_node.add_child(new_node)
            self._nodes.append(parent_node)
            self._nodes.append(new_node)

        self._buffer = ""

    def endElement(self, name):
        if (name == 'BoxInstance'
                or name == 'ChoregrapheProject'
                or name == 'ParameterValue'
                or name == 'PluginContent'):
            self._nodes.pop()
        else:
            current_subnode = self._nodes.pop()
            if self._buffer:
                current_subnode.add_content(self._buffer)

        self._buffer = ""

    def characters(self, content):
        self._buffer += content

    def get_root(self):
        return self._root
