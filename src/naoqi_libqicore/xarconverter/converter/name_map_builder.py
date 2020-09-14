#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

import string
import re
import os
import sys

import converter.node.box as node_box
import converter.node.behavior_layer as node_behavior_layer
import converter.node.behavior_keyframe as node_behavior_keyframe


def format_name(name):
    """ Format a name and erase not printable chars

        :param name: name to format
        :returns: formatted name
    """
    if not name:
        return "unnamed_object"

    result = ""
    for char in name:
        if char in string.printable:
            result += char

    name = result
    pattern = re.compile(r'[\W_]+')
    return pattern.sub('_', name)


def find_io_name(box, io_id):
    """ Find the name of the io designed by id in a box

        :param box: box with ports
        :param io_id: id of the io
        :returns: name of the io as a string
    """
    for io in box.inputs + box.outputs + box.parameters:
        if (io.id == io_id):
            return io.name
    return ""


def find_input_nature(box, io_id):
    """ Find the nature of the input designed by id in a box

        :param box: box with inputs
        :param io_id: id of the io
        :returns: nature of the io as a string
    """
    for io in box.inputs:
        if (io.id == io_id):
            return io.nature
    return ""


class NameMapBuilder:
    """ Do a traversal of xar objects and rename each objects
        according to the new format convention
    """

    def __init__(self):
        self._box_stack = []  # used for IOs
        self._boxes = {}
        self._name_set = set()

    def construct_name(self, node):
        name = node.name + str(node.id)
        if node.parent_path:
            name = node.parent_path + "_" + name

        name = format_name(name)  # to removed unwanted characters
        return self.find_available_name(name)

    def construct_node_prefix(self, node):
        prefix = ""
        if node:
            prefix = node.parent_path
            # check only nodes that can have influence on prefix
            if isinstance(node, node_box.Box):
                if prefix:
                    prefix += "_"
                prefix += str("B")
                if node.id != str(-1):
                    prefix += str(node.id)
            elif isinstance(node, node_behavior_layer.BehaviorLayer):
                if prefix:
                    prefix += "_"
                prefix += str(node.id)
            elif isinstance(node, node_behavior_keyframe.BehaviorKeyframe):
                if prefix:
                    prefix += "_"
                prefix += str(node.id)

        return prefix

    def find_available_name(self, name):
        if (name in self._name_set):
            raise Exception("Conflict found on name: " + name)
        else:
            self._name_set.add(name)
        return name

    def get_name_map(self):
        """ Returns a map with names and associated objects

            :returns: a map
        """
        return self._boxes

    def visit(self, node):
        """ Visit a node, and choose the good method to apply
        """
        if not node:
            return
        methname = "_visit_%s" % node.node_type.lower()
        method = getattr(self, methname, self.visit)
        return method(node)

    def _visit_timeline(self, node):
        # Construct parent_path for children
        node.parent_path = self.construct_node_prefix(node.parent_node)
        # Timelines have the same path + basename as the owner box
        node.node_path = node.parent_node.node_path
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_diagram(self, node):
        # diagrams have the same path as their parent
        node.parent_path = self.construct_node_prefix(node.parent_node)

        # diagrams have the same path + basename as the owner keyframe
        # but if parent timeline isn't enable
        timeline = node.parent_node.parent_node.parent_node
        if timeline.enable == "0":
            node.node_path = timeline.node_path
        else:
            node.node_path = node.parent_node.node_path

        for childNode in node.children_node:
            self.visit(childNode)

        id_map = {}
        for child in node.boxes:
            id_map[child.id] = child.node_path
        parent_name = ""
        if self._box_stack:
            parent_name = self._box_stack[len(self._box_stack) - 1].node_path
        else:
            sys.stderr.write("ERROR: No parent for " + node.name
                             + " abort..." + os.linesep)
            sys.exit(2)
        id_map[str(0)] = parent_name

        # Diagram has no name in legacy format so we fill it
        # useless ??
        # node.name = self.find_available_name(parent_name + "_diagram")

        for link in node.links:
            link.emitterName = id_map[link.emitterID]
            link.receiverName = id_map[link.receiverID]

            link.inputName = find_io_name(self._boxes[link.emitterName],
                                          link.indexofinput)
            link.outputName = find_io_name(self._boxes[link.receiverName],
                                           link.indexofoutput)

    def _visit_box(self, node):
        node.parent_path = self.construct_node_prefix(node.parent_node)
        node.node_path = self.construct_name(node)  # check collisions
        self._boxes[node.node_path] = node

        self._box_stack.append(node)
        for childNode in node.children_node:
            self.visit(childNode)
        self._box_stack.pop()

    def _visit_behaviorlayer(self, node):
        node.parent_path = self.construct_node_prefix(node.parent_node)
        node.node_path = self.construct_name(node)  # check collisions
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_behaviorkeyframe(self, node):
        node.parent_path = self.construct_node_prefix(node.parent_node)
        node.node_path = self.construct_name(node)  # check collisions
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_bitmap(self, node):
        node.parent_path = self.construct_node_prefix(node.parent_node)
        node.node_path = node.parent_path + "_Bitmap"
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_script(self, node):
        # useless
        node.parent_path = self.construct_node_prefix(node.parent_node)

        # script have the same path + basename than the owner box
        node.node_path = node.parent_node.node_path
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_parameter(self, node):
        node.parent_path = self.construct_node_prefix(node.parent_node)
        node.node_path = node.parent_path + "_Parameter"

    def _visit_plugincontent(self, node):
        node.parent_path = self.construct_node_prefix(node.parent_node)
        node.node_path = node.parent_path + "_PluginContent"

    def _visit_actuatorlist(self, node):
        node.parent_path = self.construct_node_prefix(node.parent_node)
        # ActuatorLists have the same path + basename as owner box/timeline
        node.node_path = node.parent_node.node_path
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_actuatorcurve(self, node):
        node.parent_path = self.construct_node_prefix(node.parent_node)
        node.node_path = node.parent_path + "_ActuatorCurve"
        for childNode in node.children_node:
            self.visit(childNode)

    def _visit_actuatorkey(self, node):
        node.parent_path = self.construct_node_prefix(node.parent_node)
        node.node_path = node.parent_path + "_ActuatorKey"
        for childNode in node.children_node:
            self.visit(childNode)
