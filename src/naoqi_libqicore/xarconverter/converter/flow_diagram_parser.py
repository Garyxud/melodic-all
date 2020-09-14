#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import xml.sax

import converter.xar_types as xar_types

import converter.node.flow_diagram as node_flow_diagram
import converter.node.box_instance as node_box_instance
import converter.node.parameter_value as node_parameter_value
import converter.node.plugin_content as node_plugin_content


def generate_tree_from_filename(filename):
    """ Generates the reprenstation of the file designed by filename
        in memory. Build a tree.

        :param filename: name of the file to parse
        :returns: the tree representing the file
    """
    parser = xml.sax.make_parser()
    handler = FlowDiagramHandler()
    parser.setContentHandler(handler)
    parser.parse(open(filename))

    root_node = handler.get_root()
    if root_node.format_version != "4":
        return None

    return root_node


class FlowDiagramHandler(xml.sax.handler.ContentHandler):
    """ ContentHandler to parse the flow diagram fld file
    """

    def __init__(self):
        xml.sax.handler.ContentHandler.__init__(self)
        self._nodes = []
        self._buffer = ""
        self._plugin_content = False
        self._root = None

    def startElement(self, name, attrs):
        new_node = None
        if name == 'FlowDiagram':
            new_node = node_flow_diagram.FlowDiagram(
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
            self._plugin_content = True

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
            if self._plugin_content:
                """ this beacon can embed anything, raw string as well as
                    sub nodes.
                """
                new_node = node_plugin_content.PluginSubNode(name, attrs)
                parent_node = self._nodes.pop()
                parent_node.add_subnode(new_node)
                parent_node.add_child(new_node)
                self._nodes.append(parent_node)

                self._nodes.append(new_node)
            else:
                parent_node = self._nodes.pop()
                parent_node.attach_attribute(name,
                                             xar_types.attributes(attrs))
                self._nodes.append(parent_node)

        self._buffer = ""

    def endElement(self, name):
        if (name == 'BoxInstance'
                or name == 'FlowDiagram'
                or name == 'ParameterValue'):
            self._nodes.pop()

        elif name == 'PluginContent':
            self._plugin_content = False
            self._nodes.pop()

        elif self._plugin_content:
            current_subnode = self._nodes.pop()
            if self._buffer:
                current_subnode.add_content(self._buffer)

        self._buffer = ""

    def characters(self, content):
        self._buffer += content

    def get_root(self):
        return self._root
