#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import codecs
import xml.sax

import converter.xar_types as xar_types
import converter.node.box as node_box
import converter.node.bitmap as node_bitmap
import converter.node.script as node_script
import converter.node.parameter as node_parameter
import converter.node.plugin_content as node_plugin_content
import converter.node.diagram as node_diagram
import converter.node.timeline as node_timeline
import converter.node.behavior_layer as node_behavior_layer
import converter.node.behavior_keyframe as node_behavior_keyframe
import converter.node.actuator_list as node_actuator_list
import converter.node.actuator_curve as node_actuator_curve
import converter.node.actuator_key as node_actuator_key


def generate_tree_from_filename(filename):
    """ Generates the reprenstation of the file designed by filename
        in memory. Build a tree.

        :param filename: name of the file to parse
        :returns: the tree representing the file
    """
    ofile = _check_open_file(filename)
    if not ofile:
        return None
    parser = xml.sax.make_parser()
    handler = XarHandler()
    parser.setContentHandler(handler)
    parser.parse(open(filename))
    return handler.get_root()


def _check_open_file(filename):
    with codecs.open(filename, encoding='utf-8', mode='r') as ofile:
        header = ofile.readline()
        header = header + ofile.readline()
        if (not ("xar_version=\"3\"" in header)):
            return False
        return True


class XarHandler(xml.sax.handler.ContentHandler):
    """ ContentHandler to parse the xar file
    """

    def __init__(self):
        xml.sax.handler.ContentHandler.__init__(self)
        self._nodes = []
        self._buffer = ""
        self._plugin_content = False
        self._root = None

    def startElement(self, name, attrs):
        new_node = None
        if name == 'Box':
            new_node = node_box.Box(xar_types.attributes(attrs))
        elif name == 'script':
            new_node = node_script.Script(xar_types.attributes(attrs))
        elif name == 'Parameter':
            new_node = node_parameter.Parameter(xar_types.attributes(attrs))
        elif name == 'pluginContent':
            new_node = node_plugin_content.PluginContent(
                xar_types.attributes(attrs))
            self._plugin_content = True
        elif name == 'Timeline':
            new_node = node_timeline.Timeline(xar_types.attributes(attrs))
        elif name == 'BehaviorLayer':
            new_node = node_behavior_layer.BehaviorLayer(
                xar_types.attributes(attrs))
        elif name == 'BehaviorKeyframe':
            new_node = node_behavior_keyframe.BehaviorKeyframe(
                xar_types.attributes(attrs))
        elif name == 'Diagram':
            new_node = node_diagram.Diagram(xar_types.attributes(attrs))
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
            """ pluginContent beacon can embed anything, raw string as well as
                sub nodes.
            """
            if self._plugin_content:
                new_node = node_plugin_content.PluginSubNode(name, attrs)
                parent_node = self._nodes.pop()
                parent_node.add_subnode(new_node)
                parent_node.add_child(new_node)
                self._nodes.append(parent_node)
                self._nodes.append(new_node)

            elif (self._nodes and name != 'bitmap' and name != 'content'):
                parent_node = self._nodes.pop()
                parent_node.attach_attribute(name,
                                             xar_types.attributes(attrs))
                self._nodes.append(parent_node)

        self._buffer = ""

    def endElement(self, name):
        if (name == 'Box'
                or name == 'script' or name == 'Parameter'
                or name == 'Timeline' or name == 'BehaviorLayer'
                or name == 'BehaviorKeyframe' or name == 'Diagram'
                or name == 'ActuatorList' or name == 'ActuatorCurve'
                or name == 'Key'):
            self._nodes.pop()
        elif name == 'pluginContent':
            self._plugin_content = False
            self._nodes.pop()
        elif self._plugin_content:
            current_subnode = self._nodes.pop()
            if self._buffer:
                current_subnode.add_content(self._buffer)
        elif name == 'bitmap':
            bitmap = node_bitmap.Bitmap(self._buffer)
            parent_node = self._nodes.pop()
            parent_node.attach_attribute(name, bitmap)
            parent_node.add_child(bitmap)
            self._nodes.append(parent_node)
        elif name == 'content':
            parent_node = self._nodes.pop()
            parent_node.attach_attribute(name, self._buffer)
            self._nodes.append(parent_node)

        self._buffer = ""

    def characters(self, content):
        self._buffer += content

    def get_root(self):
        return self._root
