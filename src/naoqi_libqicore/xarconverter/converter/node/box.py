#!/usr/bin/env python

## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

""" Contains a class that holds box informations
.. module:: node
"""

import converter.node as node
import converter.xar_types as xar_types


class Box(node.Node):
    """ Stores informations about box in the xar format
    """

    def __init__(self, attrs):
        super(Box, self).__init__("Box")

        self.child = None
        self.parent = None

        # Attributes
        self.name = attrs.getValue("name")
        self.robot = attrs.getValue("robot")
        self.id = attrs.getValue("id")
        self.localization = attrs.getValue("localization")
        self.tooltip = attrs.getValue("tooltip")
        self.bitmap_expanded = attrs.getValue("bitmap_expanded")
        self.plugin = attrs.getValue("plugin")
        self.x_pos = attrs.getValue("x")
        self.y_pos = attrs.getValue("y")

        # Elements
        self.bitmaps = []
        self.script = None
        self.inputs = []
        self.outputs = []
        self.parameters = []
        self.resources = []
        self.plugin_content = None
        self.timeline = None

        # Function map to speed up process
        self._function_map = {'bitmap': Box.attach_bitmap,
                              'script': Box.attach_script,
                              'Input': Box.attach_input,
                              'Output': Box.attach_output,
                              'Parameter': Box.attach_parameter,
                              'Resource': Box.attach_resource,
                              'pluginContent': Box.attach_plugin_content,
                              'Timeline': Box.attach_timeline}

    def attach_timeline(self, timeline):
        self.timeline = timeline

    def attach_bitmap(self, bitmap):
        self.bitmaps.append(bitmap)

    def attach_script(self, script):
        self.script = script

    def attach_input(self, attrs):
        ioType = attrs.getValue('type')
        ioSize = attrs.getValue('type_size')

        signature = xar_types.resolve_io_signature(ioType, ioSize)
        input = xar_types.IO(attrs.getValue('name'),
                             signature,
                             attrs.getValue('nature'),
                             attrs.getValue('stm_value_name'),
                             attrs.getValue('inner'),
                             attrs.getValue('tooltip'),
                             attrs.getValue('id'))
        self.inputs.append(input)

    def attach_output(self, attrs):
        ioType = attrs.getValue('type')
        ioSize = attrs.getValue('type_size')

        signature = xar_types.resolve_io_signature(ioType, ioSize)
        output = xar_types.IO(attrs.getValue('name'),
                              signature,
                              attrs.getValue('nature'),
                              attrs.getValue('stm_value_name'),
                              attrs.getValue('inner'),
                              attrs.getValue('tooltip'),
                              attrs.getValue('id'))
        self.outputs.append(output)

    def attach_parameter(self, parameter):
        self.parameters.append(parameter)

    def attach_resource(self, attrs):
        resource = xar_types.resource(attrs.getValue('name'),
                                      attrs.getValue('type'),
                                      attrs.getValue('timeout'))
        self.resources.append(resource)

    def attach_plugin_content(self, plugin_content):
        self.plugin_content = plugin_content
